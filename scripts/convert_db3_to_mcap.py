#!/usr/bin/env python3
"""
Convert a ROS 2 rosbag2 directory in sqlite3 (.db3) storage to MCAP format
using pure Python (no docker / ros2 CLI dependency).

Reads messages with `rosbags` (which handles the rosbag2 metadata format
and sqlite3 storage) and writes them with the low-level `mcap` library,
registering schemas via the rosbags typestore.

Usage:
  python scripts/convert_db3_to_mcap.py <input_bag_dir> <output_bag_dir>

Both arguments are directories. The input must contain `metadata.yaml`
and one or more `.db3` files. The output directory will be created and
will contain `metadata.yaml` + `<name>_0.mcap`.

Dependencies: pip install rosbags mcap
"""
from __future__ import annotations

import argparse
import sys
import shutil
from pathlib import Path

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    print("pip install rosbags", file=sys.stderr)
    raise

try:
    from mcap.writer import Writer as McapWriter
    from mcap.reader import make_reader as make_mcap_reader
except ImportError:
    print("pip install mcap", file=sys.stderr)
    raise


def _load_custom_schemas_from_source_mcap(source_bag_dir: Path) -> dict[str, bytes]:
    """If a source mcap exists (the original input bag the db3 was generated
    from), read its schemas and return a map of {msgtype_name → schema_bytes}.
    Used as a fallback for custom message types that aren't in
    rosbags' built-in typestore (e.g. ublox_ubx_msgs)."""
    out: dict[str, bytes] = {}
    # Walk up the path looking for a sibling mcap file in any parent dir
    # named .../recordings/rosbags/... and matching the bag pattern.
    candidate_roots: list[Path] = []
    cur = source_bag_dir
    for _ in range(8):
        cur = cur.parent
        if cur.name.startswith("grid_") or cur.name.startswith("rectangle_") or "2026-05-07" in str(cur):
            candidate_roots.append(cur)
    for root in candidate_roots:
        for mcap in root.glob("**/*.mcap"):
            try:
                with open(mcap, "rb") as f:
                    reader = make_mcap_reader(f)
                    for schema in reader.get_summary().schemas.values():
                        if schema.name not in out:
                            out[schema.name] = schema.data
                if out:
                    return out
            except Exception:
                continue
    return out


def _msg_text(typestore, msgtype: str, fallback_schemas: dict[str, bytes]) -> bytes:
    """Return ros2msg-format schema bytes for `msgtype`. Tries the rosbags
    built-in typestore first; falls back to a custom-schema map sourced from
    an existing mcap bag for unknown types (custom messages)."""
    try:
        defn, _hash = typestore.generate_msgdef(msgtype)
        return defn.encode("utf-8")
    except Exception:
        if msgtype in fallback_schemas:
            return fallback_schemas[msgtype]
        # Minimal placeholder — the mcap is still valid, just opaque for
        # consumers that need the schema to deserialize this topic.
        return f"# unknown message type: {msgtype}\n".encode("utf-8")


def convert(input_bag: Path, output_bag: Path) -> int:
    if not (input_bag / "metadata.yaml").exists():
        print(f"no metadata.yaml in {input_bag}", file=sys.stderr)
        return 2
    if output_bag.exists():
        print(
            f"output already exists: {output_bag} — delete it first",
            file=sys.stderr,
        )
        return 2

    output_bag.mkdir(parents=True)
    out_mcap = output_bag / f"{output_bag.name}_0.mcap"

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    fallback_schemas = _load_custom_schemas_from_source_mcap(input_bag)
    if fallback_schemas:
        print(
            f"loaded {len(fallback_schemas)} fallback schemas from source mcap"
        )

    schema_ids: dict[str, int] = {}
    channel_ids: dict[str, int] = {}
    msg_count = 0

    with open(out_mcap, "wb") as f, Reader(input_bag) as reader:
        writer = McapWriter(f)
        writer.start()
        for conn in reader.connections:
            schema_bytes = _msg_text(typestore, conn.msgtype, fallback_schemas)
            sid = writer.register_schema(
                name=conn.msgtype,
                encoding="ros2msg",
                data=schema_bytes,
            )
            schema_ids[conn.topic] = sid
            cid = writer.register_channel(
                topic=conn.topic,
                message_encoding="cdr",
                schema_id=sid,
            )
            channel_ids[conn.topic] = cid

        for conn, timestamp, rawdata in reader.messages():
            writer.add_message(
                channel_id=channel_ids[conn.topic],
                log_time=int(timestamp),
                data=bytes(rawdata),
                publish_time=int(timestamp),
            )
            msg_count += 1
            if msg_count % 50000 == 0:
                print(f"  wrote {msg_count} messages...")
        writer.finish()

    # Minimal metadata.yaml for ros2 bag play. The mcap file itself carries
    # the topic/schema/message info, but `ros2 bag play` requires a
    # metadata.yaml alongside.
    _write_metadata(input_bag, output_bag, out_mcap, msg_count)

    print(f"\nwrote {msg_count} messages to {out_mcap}")
    print(f"file size: {out_mcap.stat().st_size / 1024 / 1024:.1f} MiB")
    return 0


def _write_metadata(
    input_bag: Path, output_bag: Path, out_mcap: Path, total_messages: int
) -> None:
    """Generate a metadata.yaml matching the input's structure but pointing
    at the new mcap file with storage_id mcap."""
    import yaml

    with open(input_bag / "metadata.yaml") as f:
        meta = yaml.safe_load(f)

    info = meta["rosbag2_bagfile_information"]
    info["storage_identifier"] = "mcap"
    info["relative_file_paths"] = [out_mcap.name]
    info["files"] = [
        {
            "path": out_mcap.name,
            "starting_time": info.get("starting_time", {}),
            "duration": info.get("duration", {}),
            "message_count": total_messages,
        }
    ]
    info["message_count"] = total_messages
    info["compression_format"] = ""
    info["compression_mode"] = ""

    with open(output_bag / "metadata.yaml", "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("input_bag", help="Input rosbag2 directory (db3).")
    p.add_argument("output_bag", help="Output rosbag2 directory (mcap).")
    args = p.parse_args()
    return convert(Path(args.input_bag), Path(args.output_bag))


if __name__ == "__main__":
    sys.exit(main())
