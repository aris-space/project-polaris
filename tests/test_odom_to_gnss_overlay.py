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
