import pytest
from contracts import (
    DroneState,
    Obstacle,
    Route,
    Scenario,
    Vector3,
    VehicleInterface,
    VehicleState,
    Wind,
)
from scenario import ScenarioManager


class MockVehicle(VehicleInterface):
    """Predictable vehicle mockup for unit testing."""

    def __init__(self, position: Vector3 | None = None, payload_capacity: float = 5.0, battery: float = 100.0):
        self.state = VehicleState(
            position=position or Vector3(0, 0, 0),
            battery_percent=battery,
            state=DroneState.LANDED,
            payload_capacity=payload_capacity,
        )
        self.uploaded_route: Route | None = None

    def get_state(self) -> VehicleState:
        return self.state

    def upload_route(self, route: Route) -> None:
        self.uploaded_route = route

    def start_mission(self) -> None:
        self.state.state = DroneState.FLYING

    def pause_mission(self) -> None:
        pass

    def abort_mission(self) -> None:
        self.state.state = DroneState.EMERGENCY

    def return_to_base(self) -> None:
        self.state.state = DroneState.RETURNING

    def advance(self, dt: float) -> None:
        pass

    def reset(self) -> None:
        self.state.position = Vector3(0, 0, 0)
        self.state.battery_percent = 100.0
        self.state.state = DroneState.LANDED
        self.uploaded_route = None


@pytest.fixture
def sample_scenario():
    return Scenario(
        bounds=(0, 0, 100, 100),
        base=Vector3(0, 0, 0),
        wind=Wind(Vector3(0, 0, 0)),
        obstacles=[
            Obstacle(position=Vector3(20, 20, 0), width=4, depth=4, height=10)
        ],
        orders=[],
    )


@pytest.fixture
def scenario_manager(sample_scenario):
    return ScenarioManager(sample_scenario)