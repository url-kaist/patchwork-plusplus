import os

import numpy as np
import pypatchworkpp
import pytest

DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data")


def _read_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def test_module_imports_and_exposes_api():
    assert hasattr(pypatchworkpp, "Parameters")
    assert hasattr(pypatchworkpp, "patchworkpp")


@pytest.mark.parametrize("checked_rings", [0, 4, 6, 14])
def test_estimate_ground_partitions_all_points(checked_rings):
    params = pypatchworkpp.Parameters()
    params.num_rings_of_interest = checked_rings
    pp = pypatchworkpp.patchworkpp(params)

    scan = _read_bin(os.path.join(DATA_DIR, "000000.bin"))
    for _ in range(2):
        pp.estimateGround(scan)
        ground = pp.getGround()
        nonground = pp.getNonground()

        assert ground.shape[0] > 0
        assert nonground.shape[0] > 0
        assert ground.shape[0] + nonground.shape[0] <= scan.shape[0]


@pytest.mark.parametrize("checked_rings", [-1, 15])
def test_invalid_checked_ring_count(checked_rings):
    params = pypatchworkpp.Parameters()
    params.num_rings_of_interest = checked_rings
    with pytest.raises(ValueError, match="num_rings_of_interest"):
        pypatchworkpp.patchworkpp(params)
