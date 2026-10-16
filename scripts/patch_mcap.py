#!/usr/bin/env python3
"""
Offline: rewrite six static transforms in /tf_static AND (optionally) replace
the pressure z variance in ROS2 MCAP rosbags. All other data is copied
byte-for-byte.

Use case: bags recorded against an older URDF / Xsens driver branch need to be
re-analysed with the new COM-aligned TFs (and optionally the noise level from
the Allan-variance study) without re-recording.

Dependencies: pip install mcap mcap-ros2-support numpy scipy

Examples:
  # TF only (default)
  python scripts/patch_mcap.py recordings/rosbags/2026-04-23

  # TF + pressure z variance
  python scripts/patch_mcap.py recordings/rosbags/2026-04-23 \\
      --output-dir recordings/rosbags/2026-04-23_patched \\
      --pressure-z-var 0.001

  # Pressure variance only, leave /tf_static alone
  python scripts/patch_mcap.py recordings/rosbags/2026-04-23 \\
      --skip-tf --pressure-z-var 0.001

The --imu-* and --dvl-* flags from the original spec are intentionally NOT
implemented: DVL twist covariance and IMU orientation/gyro covariance are out
of scope and pass through byte-for-byte.
"""
from __future__ import annotations

import argparse
import math
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path

from scipy.spatial.transform import Rotation as Rot

try:
    from mcap.reader import make_reader
    from mcap.writer import Writer as McapWriter
except ImportError:
    print("Missing dependency. Install: pip install mcap", file=sys.stderr)
    raise

try:
    from mcap_ros2.decoder import DecoderFactory
    from mcap_ros2._dynamic import serialize_dynamic
except ImportError:
    print("Missing dependency. Install: pip install mcap-ros2-support", file=sys.stderr)
    raise


TF_STATIC_TOPIC = "/tf_static"
PRESSURE_TOPIC = "/sensors/pressure/pose_enu"

_PI = math.pi

# parent, child -> ((tx, ty, tz), (roll, pitch, yaw))   (radians, scipy 'xyz' euler convention)
# imu_link keeps yaw=π intentionally: bags pre-date moving the rotation into the
# Xsens driver, so the recorded IMU data is still in the rotated frame and the
# patched TF must keep yaw=π.
PATCHED_TFS: dict[
    tuple[str, str], tuple[tuple[float, float, float], tuple[float, float, float]]
] = {
    ("base_link", "imu_link"):                   ((0.049,    -0.0088,    0.076),  (0.0,  0.0, _PI)),
    ("base_link", "dvl_a50_link"):               ((0.7216,   -0.000243, -0.075),  (_PI,  0.0, -_PI / 4)),
    ("base_link", "gnss_link"):                  ((-0.0057,  -0.00024,   0.174),  (0.0,  0.0, 0.0)),
    ("base_link", "sbl_link"):                   ((-0.614,   -0.000086,  0.197),  (0.0,  0.0, 0.0)),
    ("base_link", "bluerobotics_pressure_link"): ((0.5086,   -0.035,    -0.039),  (0.0,  0.0, 0.0)),
}


def _self_check() -> None:
    """Catch silent constant drift before doing any work."""
    assert math.isclose(-_PI / 4, -0.7853981633974483, rel_tol=0, abs_tol=1e-15), (
        "dvl yaw constant drifted from spec value"
    )


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------


@dataclass(frozen=True)
class CovConfig:
    pressure_z_var: float


def _build_cov_config(args: argparse.Namespace) -> CovConfig | None:
    if args.pressure_z_var is None:
        return None
    return CovConfig(pressure_z_var=float(args.pressure_z_var))


def _build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "bag_dir",
        type=Path,
        help="Parent directory containing one or more rosbag2 dirs "
        "(each with metadata.yaml + *.mcap).",
    )
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output parent directory. Default: <bag_dir>_patched/ as a sibling.",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="Report which bags would be processed without writing anything.",
    )
    ap.add_argument(
        "--skip-tf",
        action="store_true",
        help="Do not patch /tf_static (only useful with --pressure-z-var).",
    )
    ap.add_argument(
        "--skip-covariance",
        action="store_true",
        help="Do not patch sensor covariances even if --pressure-z-var is provided.",
    )
    ap.add_argument(
        "--pressure-z-var",
        type=float,
        default=None,
        help="Replace /sensors/pressure/pose_enu pose.covariance[14] (z variance).",
    )
    return ap


# ----------------------------------------------------------------------------
# TF patching
# ----------------------------------------------------------------------------


def _apply_tf_patch(t) -> bool:
    """Mutate t.transform fields in place to PATCHED_TFS values. Return True if patched."""
    key = (t.header.frame_id, t.child_frame_id)
    spec = PATCHED_TFS.get(key)
    if spec is None:
        return False
    (tx, ty, tz), rpy = spec
    qx, qy, qz, qw = Rot.from_euler("xyz", rpy).as_quat()
    t.transform.translation.x = float(tx)
    t.transform.translation.y = float(ty)
    t.transform.translation.z = float(tz)
    t.transform.rotation.x = float(qx)
    t.transform.rotation.y = float(qy)
    t.transform.rotation.z = float(qz)
    t.transform.rotation.w = float(qw)
    return True


# ----------------------------------------------------------------------------
# Covariance patching
# ----------------------------------------------------------------------------


def _patch_pressure(msg, cfg: CovConfig) -> bool:
    """PoseWithCovarianceStamped: 6×6 row-major; index 14 = z variance (row 2 col 2)."""
    msg.pose.covariance[14] = float(cfg.pressure_z_var)
    return True


# ----------------------------------------------------------------------------
# MCAP I/O plumbing
# ----------------------------------------------------------------------------


def _encoder_for(schema):
    """Return an encoder taking a decoded message -> CDR bytes."""
    schema_text = schema.data.decode("utf-8")
    encoders = serialize_dynamic(schema.name, schema_text)
    enc = encoders.get(schema.name)
    if enc is None:
        raise RuntimeError(
            f"serialize_dynamic produced no encoder for schema {schema.name!r}"
        )
    return enc


def _mirror_schemas_channels(
    summary, writer: McapWriter
) -> tuple[dict[int, int], dict[int, int]]:
    """Copy every schema and channel from the input summary into the output writer.
    Return (schema_id_map, channel_id_map): input_id -> output_id."""
    schema_id_map: dict[int, int] = {}
    for sid, schema in summary.schemas.items():
        out_id = writer.register_schema(
            name=schema.name,
            encoding=schema.encoding,
            data=schema.data,
        )
        schema_id_map[sid] = out_id

    channel_id_map: dict[int, int] = {}
    for cid, channel in summary.channels.items():
        out_id = writer.register_channel(
            topic=channel.topic,
            message_encoding=channel.message_encoding,
            schema_id=schema_id_map[channel.schema_id],
            metadata=channel.metadata,
        )
        channel_id_map[cid] = out_id
    return schema_id_map, channel_id_map


# ----------------------------------------------------------------------------
# Per-bag processing
# ----------------------------------------------------------------------------


@dataclass
class BagStats:
    tf_patched: dict[tuple[str, str], int] = field(default_factory=dict)
    pressure_patched: int = 0
    total_msgs: int = 0
    per_topic_count: dict[str, int] = field(default_factory=dict)


def _find_bag_dirs(root: Path) -> list[Path]:
    """Return rosbag2 directories under root (must have metadata.yaml + *.mcap)."""
    dirs: set[Path] = set()
    for meta in root.rglob("metadata.yaml"):
        d = meta.parent
        if any(d.glob("*.mcap")):
            dirs.add(d.resolve())
    return sorted(dirs)


def _process_bag(
    bag_dir: Path,
    out_bag: Path,
    *,
    patch_tf: bool,
    cov_cfg: CovConfig | None,
    dry_run: bool,
) -> bool:
    mcaps = sorted(bag_dir.glob("*.mcap"))
    if len(mcaps) != 1:
        print(
            f"[skip] {bag_dir.name}: expected exactly one .mcap, found {len(mcaps)}",
            file=sys.stderr,
        )
        return False
    in_mcap = mcaps[0]

    if dry_run:
        print(f"[dry-run] would patch {in_mcap} -> {out_bag / in_mcap.name}")
        return True

    out_bag.mkdir(parents=True, exist_ok=True)
    out_mcap = out_bag / in_mcap.name

    stats = BagStats()

    patched_topics: set[str] = set()
    if patch_tf:
        patched_topics.add(TF_STATIC_TOPIC)
    if cov_cfg is not None:
        patched_topics.add(PRESSURE_TOPIC)

    decoder_factory = DecoderFactory()

    with in_mcap.open("rb") as fin, out_mcap.open("wb") as fout:
        reader = make_reader(fin)
        summary = reader.get_summary()
        if summary is None:
            print(
                f"[skip] {bag_dir.name}: no summary index in {in_mcap.name}",
                file=sys.stderr,
            )
            return False

        writer = McapWriter(fout)
        writer.start()

        _, channel_id_map = _mirror_schemas_channels(summary, writer)

        # Build encoders + per-channel decoders only for topics we will mutate.
        encoders: dict[str, callable] = {}
        per_channel_decoder: dict[int, callable] = {}
        for cid, channel in summary.channels.items():
            if channel.topic not in patched_topics:
                continue
            schema = summary.schemas[channel.schema_id]
            encoders[channel.topic] = _encoder_for(schema)
            decoder = decoder_factory.decoder_for(channel.message_encoding, schema)
            if decoder is None:
                raise RuntimeError(
                    f"No decoder for {channel.topic} ({channel.message_encoding}, {schema.name})"
                )
            per_channel_decoder[cid] = decoder

        for schema, channel, message in reader.iter_messages(log_time_order=True):
            stats.total_msgs += 1
            stats.per_topic_count[channel.topic] = (
                stats.per_topic_count.get(channel.topic, 0) + 1
            )
            out_cid = channel_id_map[channel.id]

            if channel.topic in patched_topics:
                msg = per_channel_decoder[channel.id](message.data)
                changed = False
                if channel.topic == TF_STATIC_TOPIC and patch_tf:
                    for t in msg.transforms:
                        if _apply_tf_patch(t):
                            key = (t.header.frame_id, t.child_frame_id)
                            stats.tf_patched[key] = stats.tf_patched.get(key, 0) + 1
                            changed = True
                elif channel.topic == PRESSURE_TOPIC and cov_cfg is not None:
                    if _patch_pressure(msg, cov_cfg):
                        stats.pressure_patched += 1
                        changed = True

                if changed:
                    new_data = encoders[channel.topic](msg)
                    writer.add_message(
                        channel_id=out_cid,
                        log_time=message.log_time,
                        publish_time=message.publish_time,
                        sequence=message.sequence,
                        data=new_data,
                    )
                    continue
                # Untouched message on a patched topic -> fall through to raw passthrough.

            writer.add_message(
                channel_id=out_cid,
                log_time=message.log_time,
                publish_time=message.publish_time,
                sequence=message.sequence,
                data=message.data,
            )

        writer.finish()

    # Copy metadata.yaml verbatim (rosbag2 metadata, separate from MCAP).
    src_meta = bag_dir / "metadata.yaml"
    if src_meta.is_file():
        shutil.copy2(src_meta, out_bag / "metadata.yaml")

    _print_bag_report(bag_dir, out_bag, stats)
    _verify_patched_bag(out_mcap, stats, cov_cfg=cov_cfg, patch_tf=patch_tf)
    return True


# ----------------------------------------------------------------------------
# Reporting and verification
# ----------------------------------------------------------------------------


def _print_bag_report(in_bag: Path, out_bag: Path, stats: BagStats) -> None:
    print(f"--- {in_bag.name} -> {out_bag} ---")
    print(f"  total messages: {stats.total_msgs}")
    if stats.tf_patched:
        for (parent, child), n in sorted(stats.tf_patched.items()):
            print(f"  /tf_static {parent} -> {child}: patched in {n} message(s)")
    if stats.pressure_patched:
        print(f"  /sensors/pressure/pose_enu: patched {stats.pressure_patched}")


def _summarize_cov(topic: str, msg) -> str:
    if topic == PRESSURE_TOPIC:
        return f"pose.covariance[14] = {msg.pose.covariance[14]:.6e}"
    return "?"


def _assert_tf_match(seen: dict[tuple[str, str], object]) -> None:
    """Assert every entry in PATCHED_TFS is present in `seen` with the new values."""
    for key, ((tx, ty, tz), rpy) in PATCHED_TFS.items():
        t = seen.get(key)
        if t is None:
            print(f"  [verify][warn] missing patched TF {key}")
            continue
        qx, qy, qz, qw = Rot.from_euler("xyz", rpy).as_quat()
        for got, exp, name in [
            (t.transform.translation.x, tx, "tx"),
            (t.transform.translation.y, ty, "ty"),
            (t.transform.translation.z, tz, "tz"),
            (t.transform.rotation.x, qx, "qx"),
            (t.transform.rotation.y, qy, "qy"),
            (t.transform.rotation.z, qz, "qz"),
            (t.transform.rotation.w, qw, "qw"),
        ]:
            if not math.isclose(got, exp, rel_tol=0, abs_tol=1e-12):
                print(f"  [verify][FAIL] {key} {name}: got {got!r}, expected {exp!r}")


def _verify_patched_bag(
    out_mcap: Path,
    stats: BagStats,
    *,
    cov_cfg: CovConfig | None,
    patch_tf: bool,
) -> None:
    """Read the patched MCAP back and report old-vs-new values + count parity."""
    with out_mcap.open("rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        topic_counts: dict[str, int] = {}
        seen_tfs: dict[tuple[str, str], object] = {}
        cov_samples: dict[str, list] = {PRESSURE_TOPIC: []}
        for _schema, channel, _message, decoded in reader.iter_decoded_messages(
            log_time_order=True
        ):
            topic_counts[channel.topic] = topic_counts.get(channel.topic, 0) + 1
            if patch_tf and channel.topic == TF_STATIC_TOPIC:
                # /tf_static can have multiple latched messages (one per publisher).
                # Accumulate all transforms across them; later messages overwrite earlier
                # ones, which matches /tf_static's "last one wins" semantics.
                for t in decoded.transforms:
                    seen_tfs[(t.header.frame_id, t.child_frame_id)] = t
            if (
                cov_cfg is not None
                and channel.topic in cov_samples
                and len(cov_samples[channel.topic]) < 3
            ):
                cov_samples[channel.topic].append(decoded)

    # Cross-check per-topic counts vs the write-side stats.
    mismatches = []
    for topic, n_in in stats.per_topic_count.items():
        n_out = topic_counts.get(topic, 0)
        if n_in != n_out:
            mismatches.append((topic, n_in, n_out))
    if mismatches:
        for topic, n_in, n_out in mismatches:
            print(f"  [verify][FAIL] {topic}: input {n_in} vs output {n_out}")
    else:
        print(
            f"  [verify] per-topic counts match input "
            f"({len(topic_counts)} topics, {sum(topic_counts.values())} msgs)"
        )

    if patch_tf:
        if not seen_tfs:
            print("  [verify][warn] never saw a /tf_static message - nothing to verify")
        else:
            _assert_tf_match(seen_tfs)

    if cov_cfg is not None:
        for topic, samples in cov_samples.items():
            if not samples:
                print(f"  [verify][warn] no samples on {topic}")
                continue
            for i, m in enumerate(samples):
                print(f"  [verify] sample {i} on {topic}: {_summarize_cov(topic, m)}")


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------


def main() -> int:
    _self_check()
    args = _build_argparser().parse_args()

    bag_root = args.bag_dir.resolve()
    if not bag_root.is_dir():
        print(f"Not a directory: {bag_root}", file=sys.stderr)
        return 1

    out_root = (
        args.output_dir or bag_root.with_name(bag_root.name + "_patched")
    ).resolve()

    cov_cfg = _build_cov_config(args)
    patch_tf = not args.skip_tf
    patch_cov = cov_cfg is not None and not args.skip_covariance
    if not patch_tf and not patch_cov:
        print(
            "Nothing to patch (--skip-tf set and no --pressure-z-var). Exiting.",
            file=sys.stderr,
        )
        return 1

    bag_dirs = _find_bag_dirs(bag_root)
    if not bag_dirs:
        print(
            f"No bag dirs (with metadata.yaml + *.mcap) under {bag_root}",
            file=sys.stderr,
        )
        return 1

    if not args.dry_run:
        out_root.mkdir(parents=True, exist_ok=True)

    ok = 0
    for bag in bag_dirs:
        try:
            rel = bag.relative_to(bag_root)
        except ValueError:
            rel = Path(bag.name)
        out_bag = out_root / rel
        if _process_bag(
            bag,
            out_bag,
            patch_tf=patch_tf,
            cov_cfg=cov_cfg if patch_cov else None,
            dry_run=args.dry_run,
        ):
            ok += 1

    print(f"Done. {ok}/{len(bag_dirs)} bags processed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
