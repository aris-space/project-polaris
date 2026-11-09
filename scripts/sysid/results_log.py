#!/usr/bin/env python3
"""
Append-only record of every fit: which bag (and window), which script, model and settings
produced which parameter values. One entry per fit run, in results/fit_log.yaml, committed to
git so the history survives. Entries are only ever appended; the file is never rewritten, so
comments and hand edits (for example changing `status` to accepted/rejected) are safe.

Fit scripts, after solving:
    from results_log import log_fit
    fit_id = log_fit(
        script=__file__,
        bags=[(args.bag, args.t_start, args.t_end)],       # or plain paths
        params={"m_tot": (12.1, 0.4), "X_u": 3.2, "X_uu": (18.0, 1.1)},
        units={"m_tot": "kg", "X_u": "N s/m", "X_uu": "N s^2/m^2"},
        manoeuvre="surge steps",
        model=args.model,
        settings={"smooth_s": args.smooth_s, "rate_hz": args.rate},
        metrics={"r2": r2, "residual_std_N": res_std, "n_samples": y.size},
    )
    # optionally: model_io.update_parameters({...}, path=..., bags=[...], fit_id=fit_id)

Overview (from the repo root):
    python scripts/sysid/results_log.py                 # one line per fit
    python scripts/sysid/results_log.py --param X_u     # history of one parameter
    python scripts/sysid/results_log.py --bag rectangle_08
    python scripts/sysid/results_log.py --show fit-0003 # full entry

The same machinery (_append, read_log) is meant to carry a validation log later.
"""
from __future__ import annotations

import argparse
import hashlib
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from model_io import _git_state, load_model

REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_DIR = Path(__file__).resolve().parent / "results"
DEFAULT_FIT_LOG = RESULTS_DIR / "fit_log.yaml"

_HEADER = """\
# Fit log: one entry per fit run, appended by results_log.log_fit(). Never rewritten by code.
# Overview: python scripts/sysid/results_log.py   (--param NAME, --bag TEXT, --show ID)
#
# Per entry: id, date, script (with git state), manoeuvre, bags (path, window in seconds after
# the first EKF message, size and fingerprint so a renamed or re-exported bag is recognisable),
# model (path and version tag, which includes the thrust-map hashes), settings, params
# (value, uncertainty, unit), metrics, status and notes. `status` may be edited by hand.
"""

_FINGERPRINT_CHUNK = 1 << 20  # 1 MiB from each end; bags are up to ~1 GB


# ============================================================
# Building blocks
# ============================================================


def _rel(path: Path) -> str:
    """Repo-relative POSIX path when inside the repo, else absolute."""
    path = Path(path).resolve()
    try:
        return path.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def bag_fingerprint(path: Path) -> str:
    """sha256 over the size and the first and last MiB. Fast on GB-sized bags and enough to
    tell two recordings apart; not a full content hash."""
    path = Path(path)
    h = hashlib.sha256()
    if path.is_dir():  # rosbag2 directory: fingerprint its files in name order
        for f in sorted(p for p in path.rglob("*") if p.is_file()):
            h.update(f.name.encode())
            h.update(bag_fingerprint(f).encode())
        return h.hexdigest()[:16]
    size = path.stat().st_size
    h.update(str(size).encode())
    with path.open("rb") as fh:
        h.update(fh.read(_FINGERPRINT_CHUNK))
        if size > _FINGERPRINT_CHUNK:
            fh.seek(max(_FINGERPRINT_CHUNK, size - _FINGERPRINT_CHUNK))
            h.update(fh.read())
    return h.hexdigest()[:16]


def bag_record(path: Path, t_start: float | None = None, t_end: float | None = None) -> dict:
    path = Path(path)
    entry: dict[str, Any] = {"path": _rel(path), "window_s": _plain([t_start, t_end])}
    if path.exists():
        size = path.stat().st_size if path.is_file() else sum(
            p.stat().st_size for p in path.rglob("*") if p.is_file())
        entry["size_bytes"] = size
        entry["fingerprint"] = bag_fingerprint(path)
    else:
        entry["fingerprint"] = "missing"
    return entry


def _plain(x: Any) -> Any:
    """Convert numpy scalars/arrays, Paths and tuples to plain YAML-safe types."""
    if isinstance(x, dict):
        return {str(k): _plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return _plain(x.tolist())
    if isinstance(x, np.bool_):
        return bool(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, (np.floating, float)):
        return float(x)
    if isinstance(x, Path):
        return _rel(x)
    return x


def _param_entry(value: Any, unit: str | None) -> dict:
    if isinstance(value, dict):
        out = {"value": value.get("value"), "uncertainty": value.get("uncertainty"),
               "unit": value.get("unit", unit)}
    elif isinstance(value, (tuple, list)):
        out = {"value": value[0], "uncertainty": value[1] if len(value) > 1 else None, "unit": unit}
    else:
        out = {"value": value, "uncertainty": None, "unit": unit}
    return _plain(out)


def read_log(log_path: Path = DEFAULT_FIT_LOG) -> list[dict]:
    log_path = Path(log_path)
    if not log_path.exists():
        return []
    return yaml.safe_load(log_path.read_text(encoding="utf-8")) or []


def _append(log_path: Path, prefix: str, entry: dict, header: str) -> str:
    """Give the entry the next id (<prefix>-NNNN), append it as one YAML list item, return the id."""
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    existing = read_log(log_path)
    entry_id = f"{prefix}-{len(existing) + 1:04d}"
    text = yaml.safe_dump([{"id": entry_id, **entry}], sort_keys=False,
                          allow_unicode=True, default_flow_style=None, width=200)
    with log_path.open("a", encoding="utf-8") as fh:
        if log_path.stat().st_size == 0:
            fh.write(header)
        fh.write("\n" + text)
    return entry_id


# ============================================================
# Fits
# ============================================================


def log_fit(
    script: str | Path,
    bags: Iterable[str | Path | tuple],
    params: dict[str, Any],
    *,
    units: dict[str, str] | None = None,
    manoeuvre: str = "",
    model: str | Path | None = None,
    settings: dict[str, Any] | None = None,
    metrics: dict[str, Any] | None = None,
    notes: str = "",
    status: str = "candidate",
    log_path: Path = DEFAULT_FIT_LOG,
) -> str:
    """Append one fit to the log and return its id (e.g. "fit-0007").

    bags:   paths, or (path, t_start, t_end) tuples with the window in seconds after the first
            EKF message (the --t-start/--t-end convention of the fit scripts).
    params: name -> value, (value, uncertainty) or {value, uncertainty, unit}. Names are free
            (m_tot is not a model.yaml key); for model.yaml keys ("damping_linear.X_u") the unit
            is taken from the model file when not given in `units`.
    """
    script = Path(script).resolve()
    units = dict(units or {})
    model_entry = None
    if model is not None:
        m = load_model(model)
        model_entry = {"path": _rel(m.path), "version": m.version_tag()}
        for key, p in m.params.items():
            units.setdefault(key, p.get("unit"))

    bag_entries = []
    for b in bags:
        if isinstance(b, (tuple, list)):
            bag_entries.append(bag_record(Path(b[0]), *b[1:3]))
        else:
            bag_entries.append(bag_record(Path(b)))

    entry = {
        "date": datetime.now().astimezone().isoformat(timespec="seconds"),
        "script": f"{_rel(script)} (git {_git_state(script)})",
        "manoeuvre": manoeuvre,
        "bags": bag_entries,
        "model": model_entry,
        "settings": _plain(settings or {}),
        "params": {name: _param_entry(v, units.get(name)) for name, v in params.items()},
        "metrics": _plain(metrics or {}),
        "status": status,
        "notes": notes,
    }
    return _append(log_path, "fit", entry, _HEADER)


# ============================================================
# Overview CLI
# ============================================================


def _fmt(p: dict) -> str:
    v, u = p.get("value"), p.get("uncertainty")
    if v is None:
        return "null"
    return f"{v:.4g}" + (f" +/- {u:.2g}" if u is not None else "")


def _bag_names(e: dict) -> str:
    return ", ".join(Path(b["path"]).stem[:40] for b in e.get("bags", []))


def overview(entries: list[dict], param: str | None = None, bag: str | None = None) -> str:
    if bag:
        entries = [e for e in entries if any(bag in b["path"] for b in e.get("bags", []))]
    lines = []
    if param:
        lines.append(f"{'id':<9} {'date':<10} {'status':<10} {param:<18} bag")
        for e in entries:
            p = e.get("params", {}).get(param)
            if p is not None:
                lines.append(f"{e['id']:<9} {e['date'][:10]:<10} {e.get('status', ''):<10} "
                             f"{_fmt(p):<18} {_bag_names(e)}")
    else:
        for e in entries:
            ps = "  ".join(f"{k}={_fmt(v)}" for k, v in e.get("params", {}).items())
            lines.append(f"{e['id']:<9} {e['date'][:10]}  {e.get('status', ''):<10} "
                         f"{e.get('manoeuvre', '') or '-':<14} {_bag_names(e)}\n"
                         f"{'':<9} {ps}")
    return "\n".join(lines) if lines else "(no matching entries)"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Overview of the fit log.")
    ap.add_argument("--log", type=Path, default=DEFAULT_FIT_LOG)
    ap.add_argument("--param", help="show the history of one parameter")
    ap.add_argument("--bag", help="only fits whose bag path contains this text")
    ap.add_argument("--show", metavar="ID", help="print one full entry")
    args = ap.parse_args(argv)

    entries = read_log(args.log)
    if args.show:
        hit = [e for e in entries if e["id"] == args.show]
        if not hit:
            print(f"no entry {args.show} in {args.log}")
            return 1
        print(yaml.safe_dump(hit[0], sort_keys=False, allow_unicode=True))
        return 0
    print(overview(entries, args.param, args.bag))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
