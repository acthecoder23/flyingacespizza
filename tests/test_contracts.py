import pytest
from contracts import Obstacle, Route, Vector3, Waypoint


def test_vector3_operations():
    v1 = Vector3(1.0, 2.0, 3.0)
    v2 = Vector3(4.0, 6.0, 3.0)
    
    assert v1.distance_to(v2) == 5.0
    
    v3 = v1 + v2
    assert v3 == Vector3(5.0, 8.0, 6.0)
    
    v4 = v1 * 2.0
    assert v4 == Vector3(2.0, 4.0, 6.0)


def test_route_distance():
    route = Route([
        Waypoint(Vector3(0, 0, 0)),
        Waypoint(Vector3(10, 0, 0)),
        Waypoint(Vector3(10, 10, 0)),
    ])
    assert pytest.approx(route.distance) == 20.0


def test_obstacle_contains_and_bounds():
    obs = Obstacle(position=Vector3(20, 20, 0), width=4, depth=4, height=10)
    
    # Inside obstacle bounding box
    assert obs.contains(Vector3(20, 20, 5)) is True
    assert obs.contains(Vector3(21, 21, 9)) is True
    
    # Outside obstacle bounding box
    assert obs.contains(Vector3(20, 20, 11)) is False  # Above height
    assert obs.contains(Vector3(30, 20, 5)) is False   # Outside width/depth