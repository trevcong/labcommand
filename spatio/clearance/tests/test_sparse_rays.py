"""B2: a sparse ray must not prove a whole voxel empty."""

import pytest

from spatio.clearance.__main__ import report


def zones_doc():
    return {"schema_version": 1, "frame_id": "sparse-map", "zones": [{
        "id": "door", "shape": "box", "rule_id": "example",
        "provision": "EXAMPLE, NOT APPROVED",
        "required_volume": {"min": [0, 0, 0], "max": [1, .1, .1],
                            "clearance_axis": 0, "clearance_direction": 1}}]}


def ray_scans(*rays):
    return {"schema_version": 1, "frame_id": "sparse-map",
            "scans": [{"timestamp": t, "origin": o, "hits": h} for t, o, h in rays]}


def classify_one(observations, zones=None):
    return report(zones or zones_doc(), observations, .1, [0, 0, 0], .02)["zones"][0]


def test_single_ray_does_not_prove_its_voxels_empty():
    zone = classify_one(ray_scans((1, [-.15, .05, .05], [[1.25, .05, .05]])))
    assert zone["status"] == "NOT_MEASURED"
    assert zone["insufficient_evidence_cells"] == zone["required_cells"] == 10


@pytest.mark.parametrize("separate_scans", [False, True])
@pytest.mark.parametrize("copies", [2, 5])
def test_repeated_identical_ray_counts_once(separate_scans, copies):
    origin, hit = [-.15, .05, .05], [1.25, .05, .05]
    rays = ([(i + 1, origin, [hit]) for i in range(copies)] if separate_scans
            else [(1, origin, [hit] * copies)])
    zone = classify_one(ray_scans(*rays))
    assert zone["status"] == "NOT_MEASURED"
    assert zone["insufficient_evidence_cells"] == 10


def test_corner_clipping_ray_leaves_voxel_unproven():
    zones = zones_doc()
    zones["zones"][0]["required_volume"]["max"] = [.1, .1, .1]  # one voxel
    # Clips only the voxel's top corner region, then ends far away.
    zone = classify_one(ray_scans((1, [-.05, .095, .05], [[.2, .105, .05]])), zones)
    assert zone["required_cells"] == 1
    assert zone["status"] == "NOT_MEASURED"
    assert zone["insufficient_evidence_cells"] == 1


def test_repeated_ray_hit_still_counts_as_occupancy():
    origin, hit = [-.15, .05, .05], [.85, .05, .05]
    zone = classify_one(ray_scans((1, origin, [hit]), (2, origin, [hit])))
    assert zone["status"] == "FAIL"
    assert zone["evidence_time_range"] == [1, 2]
