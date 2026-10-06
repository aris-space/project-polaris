#!/usr/bin/env python3
"""
Tests for thruster_map.ThrusterMap against the committed T500 table, and for parsing the BMS
voltage text. Grid-point values are copied from the manufacturer spreadsheet columns.

Run: python -m pytest scripts/sysid/tests/test_thruster_map.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from thruster_map import KGF_TO_N, ThrusterMap  # noqa: E402

T500 = Path(__file__).resolve().parent.parent / "thruster_curves" / "t500_force_vs_voltage.csv"


@pytest.fixture(scope="module")
def tmap() -> ThrusterMap:
    return ThrusterMap.from_csv(T500)


def test_table_shape(tmap):
    assert tmap.volts == [12.0, 14.0, 16.0, 18.0]
    assert tmap.pwm_us[0] == 1100 and tmap.pwm_us[-1] == 1900 and len(tmap.pwm_us) == 201


def test_grid_points_reproduce_the_datasheet(tmap):
    assert tmap.force_kgf_at(1900, 12) == 5.92
    assert tmap.force_kgf_at(1900, 14) == 7.31
    assert tmap.force_kgf_at(1100, 18) == -7.26


def test_neutral_and_deadband_give_zero(tmap):
    for v in (12.0, 15.2, 18.0):
        assert tmap.force_n(1500, v) == 0.0
        assert tmap.force_n(1480, v) == 0.0  # inside the tabulated zero band (1456-1544 us)


def test_linear_in_voltage_between_columns(tmap):
    assert tmap.force_kgf_at(1900, 15.0) == pytest.approx((7.31 + 9.16) / 2)


def test_linear_in_pwm_between_rows(tmap):
    lo, hi = tmap.force_kgf_at(1696, 16.0), tmap.force_kgf_at(1700, 16.0)
    assert tmap.force_kgf_at(1698, 16.0) == pytest.approx((lo + hi) / 2)


def test_newtons_are_kgf_times_g(tmap):
    assert tmap.force_n(1700, 15.2) == pytest.approx(tmap.force_kgf_at(1700, 15.2) * KGF_TO_N)


def test_out_of_range_is_clamped_and_flagged(tmap):
    assert tmap.force_n(1950, 15.0) == tmap.force_n(1900, 15.0)
    assert tmap.force_n(1700, 11.0) == tmap.force_n(1700, 12.0)
    assert tmap.clamped(1950, 15.0) and tmap.clamped(1700, 19.0)
    assert not tmap.clamped(1700, 15.0)


def test_parse_bms_voltage():
    bag_io = pytest.importorskip("bag_io")  # needs rosbags installed
    assert bag_io.parse_bms_voltage("14.70V") == 14.7
    assert bag_io.parse_bms_voltage(" 15.5 v ") == 15.5
    assert bag_io.parse_bms_voltage("n/a") is None
