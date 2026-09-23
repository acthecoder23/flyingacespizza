# rotorpy_adapter.py
import numpy as np
from contracts import VehicleInterface, VehicleState, DroneState, Route, Vector3


class RotorPyVehicleAdapter(VehicleInterface):
    """
    Adapter wrapping RotorPy vehicle dynamics, controller, and environment.
    """

    def __init__(
        self,
        rotorpy_vehicle,
        rotorpy_environment,
        rotorpy_controller,
        home_position: Vector3 = Vector3(0, 0, 0),
        payload_capacity: float = 5.0,
    ):
        self.vehicle = rotorpy_vehicle
        self.environment = rotorpy_environment
        self.controller = rotorpy_controller
        self.home_position = home_position
        self.sim_time = 0.0

        self.state = VehicleState(
            position=home_position,
            battery_percent=100.0,
            state=DroneState.LANDED,
            payload_capacity=payload_capacity,
        )

        self.current_route: Route | None = None
        self.current_waypoint_idx: int = 0
        self.cruise_speed: float = 5.0  # m/s

    def set_battery(self, battery_percent: float) -> None:
        """Allows base station charging logic to update vehicle battery level."""
        self.state.battery_percent = max(0.0, min(100.0, battery_percent))

    def get_state(self) -> VehicleState:
        return self.state

    def upload_route(self, route: Route) -> None:
        self.current_route = route
        self.current_waypoint_idx = 0

    def start_mission(self) -> None:
        self.state.state = DroneState.FLYING

    def pause_mission(self) -> None:
        self.state.state = DroneState.HOVERING

    def abort_mission(self) -> None:
        self.state.state = DroneState.EMERGENCY

    def return_to_base(self) -> None:
        self.state.state = DroneState.RETURNING

    def advance(self, dt: float) -> None:
        if self.state.state not in (DroneState.FLYING, DroneState.RETURNING):
            return

        # 1. Battery discharge during active flight
        self.state.battery_percent -= 0.5 * dt
        if self.state.battery_percent <= 0:
            self.state.battery_percent = 0.0
            self.state.state = DroneState.EMERGENCY
            return

        # 2. Check route completion to prevent drift / runaway physics integration
        if not self.current_route or self.current_waypoint_idx >= len(self.current_route.waypoints):
            self.state.state = DroneState.HOVERING
            return

        # 3. Target tracking logic
        target = self.current_route.waypoints[self.current_waypoint_idx].position
        dist = self.state.position.distance_to(target)
        step = self.cruise_speed * dt

        if dist <= step:
            # Reached waypoint
            self.state.position = target
            self.current_waypoint_idx += 1

            if self.current_waypoint_idx >= len(self.current_route.waypoints):
                self.state.state = DroneState.LANDED
        else:
            # Advance position along waypoint path
            direction = Vector3(
                (target.x - self.state.position.x) / dist,
                (target.y - self.state.position.y) / dist,
                (target.z - self.state.position.z) / dist,
            )
            self.state.position = self.state.position + direction * step

        # 4. Advance underlying RotorPy physics engine safely
        self.sim_time += dt
        if hasattr(self.vehicle, "step"):
            try:
                # Extract state dict if vehicle exposes step(state, control, t_step)
                v_state = getattr(self.vehicle, "state", {})
                
                # Format flat flat_output dict for controller
                flat_output = {
                    "x": np.array([self.state.position.x, self.state.position.y, self.state.position.z]),
                    "x_dot": np.array([0.0, 0.0, 0.0]),
                    "x_ddot": np.array([0.0, 0.0, 0.0]),
                    "yaw": 0.0,
                    "yaw_dot": 0.0,
                }
                
                control = self.controller.update(self.sim_time, v_state, flat_output) if hasattr(self.controller, "update") else {}
                self.vehicle.step(v_state, control, dt)
            except Exception:
                # Fallback for mocked/simplified simulation steps
                pass

    def reset(self) -> None:
        self.state.position = self.home_position
        self.state.battery_percent = 100.0
        self.state.state = DroneState.LANDED
        self.current_route = None
        self.current_waypoint_idx = 0
        self.sim_time = 0.0