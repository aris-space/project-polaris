import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import pytest

def test_imports():
    import odom_to_gnss_overlay  # noqa: F401

def test_argparse_defaults(tmp_path):
    from odom_to_gnss_overlay import _parse_args
    args = _parse_args([str(tmp_path)])
    assert args.max_h_acc == 2.0
    assert args.output_dir is None

def test_argparse_custom(tmp_path):
    from odom_to_gnss_overlay import _parse_args
    args = _parse_args([str(tmp_path), "--max-h-acc", "1.5", "--output-dir", str(tmp_path)])
    assert args.max_h_acc == 1.5
    assert args.output_dir == tmp_path


import math
from odom_to_gnss_overlay import (
    _h_acc_from_navsatfix,
    _h_acc_from_ubx,
    _quat_to_yaw,
    _wrap_pi,
    _compute_psi,
    _odom_to_utm,
    _COV_UNKNOWN,
    _COV_DIAGONAL,
    _COV_KNOWN,
    FixMsg,
    UbxHpMsg,
)


class TestHAccFromNavsatfix:
    def test_unknown_cov_type_returns_none(self):
        cov = [0.0] * 9
        assert _h_acc_from_navsatfix(cov, _COV_UNKNOWN) is None

    def test_diagonal_type_returns_max_std(self):
        # P_EE=4, P_NN=9 → max std = 3.0
        cov = [4.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 1.0]
        result = _h_acc_from_navsatfix(cov, _COV_DIAGONAL)
        assert result == pytest.approx(3.0, rel=1e-6)

    def test_known_type_returns_max_eigenvalue_sqrt(self):
        # Diagonal cov block: [[4, 0], [0, 4]] → eigenvalue = 4, sqrt = 2.0
        cov = [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 1.0]
        result = _h_acc_from_navsatfix(cov, _COV_KNOWN)
        assert result == pytest.approx(2.0, rel=1e-6)

    def test_known_type_off_diagonal(self):
        # [[5, 3], [3, 5]] → trace=10, det=16, eigenvalues=8,2 → sqrt(8)
        cov = [5.0, 3.0, 0.0, 3.0, 5.0, 0.0, 0.0, 0.0, 1.0]
        result = _h_acc_from_navsatfix(cov, _COV_KNOWN)
        assert result == pytest.approx(math.sqrt(8.0), rel=1e-6)


class TestHAccFromUbx:
    def test_converts_0_1mm_units_to_meters(self):
        # 10000 units * 1e-4 = 1.0 m
        assert _h_acc_from_ubx(10000) == pytest.approx(1.0, rel=1e-6)

    def test_sentinel_value_returns_none(self):
        assert _h_acc_from_ubx(0xFFFFFFFF) is None


class TestQuatToYaw:
    def test_identity_quaternion_gives_zero_yaw(self):
        assert _quat_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-9)

    def test_90deg_z_rotation(self):
        # 90° around Z: q = (0, 0, sin(π/4), cos(π/4))
        q = math.sin(math.pi / 4)
        assert _quat_to_yaw(0.0, 0.0, q, q) == pytest.approx(math.pi / 2, rel=1e-6)

    def test_negative_90deg_z_rotation(self):
        q = math.sin(math.pi / 4)
        assert _quat_to_yaw(0.0, 0.0, -q, q) == pytest.approx(-math.pi / 2, rel=1e-6)


class TestWrapPi:
    def test_no_wrap_needed(self):
        assert _wrap_pi(1.0) == pytest.approx(1.0)

    def test_wraps_above_pi(self):
        assert _wrap_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1, rel=1e-6)

    def test_wraps_below_neg_pi(self):
        assert _wrap_pi(-math.pi - 0.1) == pytest.approx(math.pi - 0.1, rel=1e-6)


class TestComputePsi:
    def test_psi_with_zero_imu_yaw(self):
        # θ_imu=0 → θ_base=0+π=π → ψ=π+π/2+_MAG_DECL, wrapped to [-π,π]
        expected = _wrap_pi(math.pi + math.pi / 2 + 0.058725188)
        assert _compute_psi(0.0) == pytest.approx(expected, rel=1e-9)


class TestOdomToUtm:
    def test_zero_displacement_stays_at_datum(self):
        E, N = _odom_to_utm(0.0, 0.0, 1000.0, 2000.0, 0.0)
        assert E == pytest.approx(1000.0)
        assert N == pytest.approx(2000.0)

    def test_x_displacement_with_psi_zero(self):
        # ψ=0: E = E0 + cos(0)*x - sin(0)*y = E0 + x
        #      N = N0 + sin(0)*x + cos(0)*y = N0 + y
        E, N = _odom_to_utm(10.0, 5.0, 1000.0, 2000.0, 0.0)
        assert E == pytest.approx(1010.0, rel=1e-9)
        assert N == pytest.approx(2005.0, rel=1e-9)

    def test_x_displacement_with_psi_90deg(self):
        # ψ=π/2: E = E0 + cos(π/2)*x - sin(π/2)*y = E0 - y
        #        N = N0 + sin(π/2)*x + cos(π/2)*y = N0 + x
        E, N = _odom_to_utm(10.0, 0.0, 1000.0, 2000.0, math.pi / 2)
        assert E == pytest.approx(1000.0, abs=1e-9)
        assert N == pytest.approx(2010.0, rel=1e-9)


from odom_to_gnss_overlay import merge_h_acc, gate_fixes


class TestGateFixes:
    def _make_fix(self, t_ns, lat=47.0, lon=8.0, status=0, cov=None, cov_type=_COV_KNOWN):
        if cov is None:
            cov = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return FixMsg(t_ns=t_ns, lat=lat, lon=lon, status=status, cov=cov, cov_type=cov_type)

    def test_rejects_no_fix_status(self):
        fixes = [self._make_fix(1000, status=-1)]
        assert gate_fixes(fixes, [0.5], max_h_acc_m=2.0) == []

    def test_rejects_large_h_acc(self):
        fixes = [self._make_fix(1000, status=0)]
        assert gate_fixes(fixes, [3.0], max_h_acc_m=2.0) == []

    def test_rejects_none_h_acc(self):
        fixes = [self._make_fix(1000, status=0, cov_type=_COV_UNKNOWN)]
        assert gate_fixes(fixes, [None], max_h_acc_m=2.0) == []

    def test_accepts_good_fix(self):
        fixes = [self._make_fix(1000, status=0)]
        accepted = gate_fixes(fixes, [1.0], max_h_acc_m=2.0)
        assert len(accepted) == 1
        assert accepted[0][0].t_ns == 1000


class TestMergeHAcc:
    def test_prefers_ubx_within_50ms(self):
        # UBX 10 ms after fix → 0.5 m (5000 * 1e-4)
        fixes = [FixMsg(t_ns=1_000_000_000, lat=0, lon=0, status=0,
                        cov=[4.0, 0, 0, 0, 4.0, 0, 0, 0, 0], cov_type=_COV_KNOWN)]
        ubx = [UbxHpMsg(t_ns=1_000_000_000 + 10_000_000, h_acc_raw=5000)]
        assert merge_h_acc(fixes, ubx)[0] == pytest.approx(0.5, rel=1e-6)

    def test_falls_back_to_covariance_when_no_ubx(self):
        # cov [[4,0],[0,4]] → eigenvalue=4 → sqrt(4)=2.0
        fixes = [FixMsg(t_ns=1_000_000_000, lat=0, lon=0, status=0,
                        cov=[4.0, 0, 0, 0, 4.0, 0, 0, 0, 0], cov_type=_COV_KNOWN)]
        assert merge_h_acc(fixes, [])[0] == pytest.approx(2.0, rel=1e-6)

    def test_sentinel_ubx_falls_back_to_covariance(self):
        fixes = [FixMsg(t_ns=1_000_000_000, lat=0, lon=0, status=0,
                        cov=[4.0, 0, 0, 0, 4.0, 0, 0, 0, 0], cov_type=_COV_KNOWN)]
        ubx = [UbxHpMsg(t_ns=1_000_000_000 + 5_000_000, h_acc_raw=0xFFFFFFFF)]
        assert merge_h_acc(fixes, ubx)[0] == pytest.approx(2.0, rel=1e-6)
