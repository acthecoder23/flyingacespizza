from autopilot_adapter import AutopilotAdapter
from contracts import DroneState, Route, Vector3, Waypoint

import pytest

def test_autopilot_adapter_route_following():
    adapter = AutopilotAdapter(speed=5.0)
    route = Route([
        Waypoint(Vector3(0, 0, 0)),
        Waypoint(Vector3(10, 0, 0)),
    ])
    
    adapter.upload_route(route)
    adapter.start_mission()
    
    assert adapter.get_state().state == DroneState.FLYING
    
    # Advance 1 second at 5 m/s speed
    adapter.advance(1.0)
    state = adapter.get_state()
    
    assert pytest.approx(state.position.x) == 5.0
    assert pytest.approx(state.position.y) == 0.0