#!/usr/bin/env python3
"""
Tests for results_log.py: entries are appended with increasing ids, numpy values come out as
plain YAML, hand edits and comments survive further appends, bag fingerprints identify a file,
units are filled from the model, and update_parameters records the fit id.

Run: python -m pytest scripts/sysid/tests/test_results_log.py
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

SYSID = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SYSID))

import results_log as rl  # noqa: E402
from model_io import load_model, update_parameters  # noqa: E402


def _bag(tmp_path: Path, name: str, content: bytes) -> Path:
    p = tmp_path / name
    p.write_bytes(content)
    return p


def test_append_ids_and_plain_types(tmp_path):
    log = tmp_path / "fit_log.yaml"
    bag = _bag(tmp_path, "a.mcap", b"x" * 5000)
    id1 = rl.log_fit(__file__, [(bag, np.float64(10.0), 60.0)],
                     {"m_tot": (np.float64(12.5), np.float32(0.5)), "X_u": np.float64(3.0)},
                     units={"m_tot": "kg"}, metrics={"n": np.int64(400), "r2": np.float64(0.9)},
                     settings={"window": np.array([1.0, 2.0])}, log_path=log)
    id2 = rl.log_fit(__file__, [bag], {"X_u": 3.1}, log_path=log)
    assert (id1, id2) == ("fit-0001", "fit-0002")

    text = log.read_text(encoding="utf-8")
    assert "numpy" not in text and "!!" not in text  # no python/numpy tags
    entries = rl.read_log(log)
    e = entries[0]
    assert e["params"]["m_tot"] == {"value": 12.5, "uncertainty": 0.5, "unit": "kg"}
    assert e["params"]["X_u"]["uncertainty"] is None
    assert e["bags"][0]["window_s"] == [10.0, 60.0]
    assert e["bags"][0]["size_bytes"] == 5000
    assert e["metrics"] == {"n": 400, "r2": 0.9}
    assert e["settings"]["window"] == [1.0, 2.0]
    assert e["status"] == "candidate"
    assert entries[1]["bags"][0]["window_s"] == [None, None]


def test_hand_edits_and_comments_survive(tmp_path):
    log = tmp_path / "fit_log.yaml"
    bag = _bag(tmp_path, "a.mcap", b"abc")
    rl.log_fit(__file__, [bag], {"X_u": 1.0}, log_path=log)
    assert log.read_text(encoding="utf-8").startswith("# Fit log")
    edited = log.read_text(encoding="utf-8").replace("status: candidate", "status: rejected  # EKF jump")
    log.write_text(edited, encoding="utf-8")
    rl.log_fit(__file__, [bag], {"X_u": 2.0}, log_path=log)
    text = log.read_text(encoding="utf-8")
    assert "# EKF jump" in text
    assert [e["status"] for e in rl.read_log(log)] == ["rejected", "candidate"]


def test_fingerprint_distinguishes_files(tmp_path):
    big = 3 * rl._FINGERPRINT_CHUNK
    a = _bag(tmp_path, "a.mcap", b"\0" * big)
    b = _bag(tmp_path, "b.mcap", b"\0" * (big - 1) + b"\1")  # differs only in the last byte
    c = _bag(tmp_path, "c.mcap", b"\0" * big)
    assert rl.bag_fingerprint(a) == rl.bag_fingerprint(c)
    assert rl.bag_fingerprint(a) != rl.bag_fingerprint(b)
    assert rl.bag_record(tmp_path / "gone.mcap")["fingerprint"] == "missing"


def test_model_units_version_and_update_link(tmp_path):
    model = tmp_path / "model.yaml"
    shutil.copy(SYSID / "model.yaml", model)
    shutil.copytree(SYSID / "thruster_curves", tmp_path / "thruster_curves")
    log = tmp_path / "fit_log.yaml"
    bag = _bag(tmp_path, "a.mcap", b"abc")
    fid = rl.log_fit(__file__, [bag], {"damping_linear.X_u": (3.0, 0.2)}, model=model, log_path=log)
    e = rl.read_log(log)[0]
    assert e["params"]["damping_linear.X_u"]["unit"] == "N s/m"
    assert e["model"]["version"].startswith("polaris_sysid v")

    update_parameters({"damping_linear.X_u": (3.0, 0.2)}, path=model, bags=["a"], fit_id=fid)
    doc = yaml.safe_load(model.read_text(encoding="utf-8"))
    assert doc["model"]["fit_ids"] == [fid]
    assert load_model(model).get("damping_linear.X_u") == 3.0


def test_overview_cli(tmp_path, capsys):
    log = tmp_path / "fit_log.yaml"
    bag = _bag(tmp_path, "zermatt_rectangle_08.mcap", b"abc")
    other = _bag(tmp_path, "grid_01.mcap", b"def")
    rl.log_fit(__file__, [bag], {"X_u": (3.0, 0.1)}, manoeuvre="surge steps", log_path=log)
    rl.log_fit(__file__, [other], {"X_u": 4.0, "m_tot": 9.0}, log_path=log)

    assert rl.main(["--log", str(log)]) == 0
    out = capsys.readouterr().out
    assert "fit-0001" in out and "fit-0002" in out and "X_u=3 +/- 0.1" in out

    rl.main(["--log", str(log), "--param", "m_tot"])
    out = capsys.readouterr().out
    assert "fit-0002" in out and "fit-0001" not in out

    rl.main(["--log", str(log), "--bag", "rectangle"])
    out = capsys.readouterr().out
    assert "fit-0001" in out and "fit-0002" not in out

    assert rl.main(["--log", str(log), "--show", "fit-0002"]) == 0
    assert "grid_01.mcap" in capsys.readouterr().out
    assert rl.main(["--log", str(log), "--show", "fit-0099"]) == 1
