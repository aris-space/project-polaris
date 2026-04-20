import json
import os
import re
import signal
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_OUTPUT_DIR_PLACEHOLDER = "__USE_NODE_DEFAULT_OUTPUT_DIR__"


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sanitize_name(value: str) -> str:
    clean = re.sub(r"[^a-zA-Z0-9_-]+", "_", value.strip())
    return clean.strip("_") or "unknown"


class RecorderControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("recorder_controller_node")

        self.command_topic = self.declare_parameter(
            "command_topic", "/polaris/recorder/command"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/polaris/recorder/status"
        ).value
        self.base_output_dir = Path(
            self.declare_parameter("base_output_dir", "~/polaris_bags").value
        ).expanduser()

        self.command_sub = self.create_subscription(String, self.command_topic, self.on_command, 20)
        self.status_pub = self.create_publisher(String, self.status_topic, 20)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.record_process: Optional[subprocess.Popen] = None
        self.recording_id: Optional[str] = None
        self.output_path: Optional[Path] = None
        self.started_at: Optional[str] = None
        self.active_event: Optional[str] = None
        self.metadata: Dict[str, str] = {"name": "", "testname": "", "location": ""}
        self.recording_options: Dict[str, Any] = {
            "mode": "all",
            "include_topics": [],
            "exclude_topics": [],
        }
        self.last_error: Optional[str] = None

        self.base_output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f"Recorder controller ready. command={self.command_topic} status={self.status_topic}"
        )

    def on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_error = "Invalid JSON command payload"
            self.get_logger().error(self.last_error)
            self.publish_status()
            return

        command = str(payload.get("command", "")).strip()
        metadata = payload.get("metadata") or {}
        if isinstance(metadata, dict):
            self.metadata = {
                "name": str(metadata.get("name", "")).strip(),
                "testname": str(metadata.get("testname", "")).strip(),
                "location": str(metadata.get("location", "")).strip(),
            }

        requested_output_dir = payload.get("base_output_dir")
        if isinstance(requested_output_dir, str):
            cleaned_output_dir = requested_output_dir.strip()
            if cleaned_output_dir and cleaned_output_dir != DEFAULT_OUTPUT_DIR_PLACEHOLDER:
                self.base_output_dir = Path(cleaned_output_dir).expanduser()
                self.base_output_dir.mkdir(parents=True, exist_ok=True)

        recording_options = payload.get("recording_options") or {}
        if isinstance(recording_options, dict):
            mode = str(recording_options.get("mode", "all")).strip().lower()
            include_topics = self.normalize_topic_list(recording_options.get("include_topics", []))
            exclude_topics = self.normalize_topic_list(recording_options.get("exclude_topics", []))
            if mode == "list":
                mode = "selection"
            self.recording_options = {
                "mode": mode if mode in {"all", "selection", "exclude_selection"} else "all",
                "include_topics": include_topics,
                "exclude_topics": exclude_topics,
            }

        requested_event = payload.get("Event", payload.get("event_name", ""))
        event_name = str(requested_event).strip() if requested_event is not None else ""

        handlers = {
            "start_recording": lambda: self.start_recording(),
            "stop_recording": lambda: self.stop_recording(),
            "add_instant_event": lambda: self.add_instant_event(event_name),
            "start_event": lambda: self.start_event(event_name),
            "stop_event": lambda: self.stop_event(event_name),
            "set_metadata": lambda: self.set_metadata(),
        }

        handler = handlers.get(command)
        if handler is None:
            self.last_error = f"Unknown command: {command}"
            self.get_logger().error(self.last_error)
            self.publish_status()
            return

        try:
            handler()
            self.last_error = None
        except Exception as exc:  # noqa: BLE001
            self.last_error = str(exc)
            self.get_logger().error(f"Command failed ({command}): {exc}")

        self.publish_status()

    def start_recording(self) -> None:
        if self.is_recording():
            raise RuntimeError("Recording already active")

        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.recording_id = self.build_recording_id(stamp)
        self.output_path = self.base_output_dir / self.recording_id
        self.output_path.mkdir(parents=True, exist_ok=True)
        self.started_at = iso_now()
        self.active_event = None

        metadata_file = self.output_path / "recording_metadata.json"
        metadata_file.write_text(
            json.dumps(
                {
                    "recording_id": self.recording_id,
                    "created_at": self.started_at,
                    "metadata": self.metadata,
                },
                indent=2,
            )
        )

        cmd = self.build_record_command(str(self.output_path / "bag"))
        self.record_process = subprocess.Popen(  # noqa: S603
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.get_logger().info(f"Started recording: {self.output_path}")
        self.get_logger().info(f"Recording command: {' '.join(cmd)}")

    def stop_recording(self) -> None:
        if not self.is_recording():
            raise RuntimeError("No recording is active")

        self.stop_record_process()
        self.active_event = None
        self.get_logger().info("Stopped recording")

    def stop_record_process(self, timeout: float = 10.0) -> None:
        process = self.record_process
        if process is None:
            return

        try:
            if process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait(timeout=timeout)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait(timeout=timeout)
                except Exception:  # noqa: BLE001
                    self.get_logger().warning(
                        "Recorder process did not exit cleanly after stop request"
                    )
        finally:
            self.record_process = None

    def add_instant_event(self, event_name: str) -> None:
        event: Dict[str, Any] = {"type": "instant", "timestamp": iso_now()}
        if event_name:
            event["Event"] = event_name
        self.write_event(event)
        if event_name:
            self.get_logger().info(f"Instant event: {event_name}")
        else:
            self.get_logger().info("Instant event")

    def start_event(self, event_name: str) -> None:
        if self.active_event is not None:
            raise RuntimeError(f"Event already active: {self.active_event}")

        self.active_event = event_name
        event: Dict[str, Any] = {"type": "start", "timestamp": iso_now()}
        if event_name:
            event["Event"] = event_name
        self.write_event(event)
        if event_name:
            self.get_logger().info(f"Started long event: {event_name}")
        else:
            self.get_logger().info("Started long event")

    def stop_event(self, event_name: str) -> None:
        if self.active_event is None:
            raise RuntimeError("No active event to stop")
        if event_name and event_name != self.active_event:
            raise RuntimeError(
                f"Active event is '{self.active_event}', but stop request was '{event_name}'"
            )

        active = self.active_event
        self.active_event = None
        event: Dict[str, Any] = {"type": "stop", "timestamp": iso_now()}
        if active:
            event["Event"] = active
        self.write_event(event)
        if active:
            self.get_logger().info(f"Stopped long event: {active}")
        else:
            self.get_logger().info("Stopped long event")

    def set_metadata(self) -> None:
        self.get_logger().info("Updated recorder metadata")

    def normalize_topic_list(self, topics: Any) -> list[str]:
        if not isinstance(topics, list):
            return []
        normalized: list[str] = []
        for topic in topics:
            if not isinstance(topic, str):
                continue
            cleaned = topic.strip()
            if not cleaned:
                continue
            if not cleaned.startswith("/"):
                cleaned = f"/{cleaned}"
            if cleaned not in normalized:
                normalized.append(cleaned)
        return normalized

    def build_record_command(self, output_bag_path: str) -> list[str]:
        mode = str(self.recording_options.get("mode", "all"))
        include_topics = self.normalize_topic_list(self.recording_options.get("include_topics", []))
        exclude_topics = self.normalize_topic_list(self.recording_options.get("exclude_topics", []))

        if mode == "selection":
            if not include_topics:
                raise RuntimeError("Record selection mode requires at least one topic")
            cmd = ["ros2", "bag", "record", *include_topics, "-o", output_bag_path]
        elif mode == "exclude_selection":
            cmd = ["ros2", "bag", "record", "-a", "-o", output_bag_path]
            if exclude_topics:
                exclude_pattern = "^(?:" + "|".join(re.escape(topic) for topic in exclude_topics) + ")$"
                cmd.extend(["--exclude", exclude_pattern])
        else:
            cmd = ["ros2", "bag", "record", "-a", "-o", output_bag_path]

        return cmd

    def build_recording_id(self, stamp: str) -> str:
        parts = [
            sanitize_name(self.metadata.get("name", "")),
            sanitize_name(self.metadata.get("testname", "")),
            sanitize_name(self.metadata.get("location", "")),
            stamp,
        ]
        return "__".join(parts)

    def write_event(self, event: Dict[str, Any]) -> None:
        if not self.output_path:
            return
        events_file = self.output_path / "events.jsonl"
        with events_file.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(event) + "\n")

    def is_recording(self) -> bool:
        return self.record_process is not None and self.record_process.poll() is None

    def publish_status(self) -> None:
        status = {
            "is_recording": self.is_recording(),
            "active_event": self.active_event,
            "recording_id": self.recording_id,
            "output_path": str(self.output_path) if self.output_path else None,
            "base_output_dir": str(self.base_output_dir),
            "started_at": self.started_at,
            "metadata": self.metadata,
            "recording_options": self.recording_options,
            "last_error": self.last_error,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def destroy_node(self) -> bool:
        self.stop_record_process(timeout=5.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RecorderControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
