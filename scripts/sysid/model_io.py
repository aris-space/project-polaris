#!/usr/bin/env python3
"""
Load, query and update the identified POLARIS model stored in model.yaml.

Validation scripts:
    model = load_model()
    X_u = model.get("damping_linear.X_u")      # raises if the value is still null
    print(model.version_tag())                 # e.g. "polaris_sysid v0.0.1 (sha256 1a2b3c4d, git 104e240f)"

Fit scripts:
    update_parameters({"damping_linear.X_u": (12.3, 0.8)}, bags=["lake_step_01"])
    # (value, uncertainty) per parameter; bumps the patch version and records the bags.

Dependency: pip install pyyaml
"""
from __future__ import annotations

import hashlib
import subprocess
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path
from typing import Any

import yaml

DEFAULT_MODEL_PATH = Path(__file__).with_name("model.yaml")


@dataclass
class Model:
    path: Path
    meta: dict[str, Any]
    params: dict[str, dict[str, Any]]  # "group.name" -> {value, unit, source, uncertainty}
    thrust: dict[str, Any] = field(default_factory=dict)
    dofs: dict[str, str] = field(default_factory=dict)
    sha256: str = ""
    git: str = field(default="unknown")

    def get(self, key: str) -> float:
        value = self.params[key]["value"]
        if value is None:
            raise ValueError(f"Model parameter '{key}' has not been determined yet ({self.path.name}).")
        return float(value)

    def missing(self) -> list[str]:
        return [k for k, p in self.params.items() if p["value"] is None]

    def missing_for(self, keys: list[str]) -> list[str]:
        """Subset of `keys` that is still null - used to fail fast with a readable list."""
        return [k for k in keys if self.params.get(k, {}).get("value") is None]

    def curve_path(self) -> Path:
        """Thruster curve CSV, resolved relative to the model file."""
        rel = self.thrust.get("curve_file")
        if not rel:
            raise ValueError(f"{self.path.name} has no thrust.curve_file entry.")
        return (self.path.parent / rel).resolve()

    def curve_sha256(self) -> str:
        try:
            return hashlib.sha256(self.curve_path().read_bytes()).hexdigest()
        except (OSError, ValueError):
            return "missing"

    def version_tag(self) -> str:
        tag = f"{self.meta['name']} v{self.meta['version']} (sha256 {self.sha256[:8]}, git {self.git}"
        if self.thrust.get("curve_file"):
            tag += f", curve {self.curve_sha256()[:8]}"
        return tag + ")"


def _git_state(path: Path) -> str:
    try:
        cwd = path.parent
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short=8", "HEAD"], cwd=cwd, text=True, stderr=subprocess.DEVNULL
        ).strip()
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain", "--", path.name], cwd=cwd, text=True, stderr=subprocess.DEVNULL
        ).strip()
        return commit + ("+dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def load_model(path: str | Path = DEFAULT_MODEL_PATH) -> Model:
    path = Path(path)
    raw = path.read_bytes()
    doc = yaml.safe_load(raw)
    params = {
        f"{group}.{name}": entry
        for group, entries in doc["parameters"].items()
        for name, entry in entries.items()
    }
    return Model(
        path=path,
        meta=doc["model"],
        params=params,
        thrust=doc.get("thrust", {}),
        dofs=doc.get("dofs", {}),
        sha256=hashlib.sha256(raw).hexdigest(),
        git=_git_state(path),
    )


def _bump_patch(version: str) -> str:
    major, minor, patch = (int(x) for x in version.split("."))
    return f"{major}.{minor}.{patch + 1}"


def update_parameters(
    updates: dict[str, tuple[float, float | None] | float],
    path: str | Path = DEFAULT_MODEL_PATH,
    bags: list[str] | None = None,
    source: str = "fitted",
    note: str | None = None,
) -> None:
    """Write new values into the model file, bump the patch version and record the bags used."""
    path = Path(path)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    for key, new in updates.items():
        group, name = key.split(".", 1)
        value, uncertainty = new if isinstance(new, tuple) else (new, None)
        entry = doc["parameters"][group][name]
        entry["value"] = float(value)
        entry["uncertainty"] = None if uncertainty is None else float(uncertainty)
        entry["source"] = source
    meta = doc["model"]
    meta["version"] = _bump_patch(str(meta["version"]))
    meta["date"] = date.today().isoformat()
    for bag in bags or []:
        if bag not in meta["fitted_from"]:
            meta["fitted_from"].append(bag)
    if note:
        meta["notes"] = note
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")


if __name__ == "__main__":
    m = load_model()
    print(m.version_tag())
    print(f"{len(m.params) - len(m.missing())}/{len(m.params)} parameters determined")
