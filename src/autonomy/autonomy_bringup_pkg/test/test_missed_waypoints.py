"""FollowWaypoints can end as SUCCEEDED although waypoints were skipped (stop_on_failure: False)."""
from types import SimpleNamespace

from autonomy_bringup_pkg.WGS84_mission_starter import missed_waypoint_indices
from nav2_msgs.action import FollowWaypoints


def test_nothing_missed():
    assert missed_waypoint_indices(SimpleNamespace(missed_waypoints=[])) == []


def test_missed_indices_are_returned_as_ints():
    assert missed_waypoint_indices(SimpleNamespace(missed_waypoints=[0, 2])) == [0, 2]


def test_result_without_the_field():
    assert missed_waypoint_indices(SimpleNamespace()) == []
    assert missed_waypoint_indices(None) == []


def test_real_action_result_type():
    result = FollowWaypoints.Result()
    assert missed_waypoint_indices(result) == []
    result.missed_waypoints = [1]
    assert missed_waypoint_indices(result) == [1]
