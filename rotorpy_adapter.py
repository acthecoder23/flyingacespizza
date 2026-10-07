# rotorpy_adapter.py
import numpy as np
# from rotorpy import wind
from contracts import VehicleInterface, VehicleState, DroneState, Route, Vector3, Wind
from battery import Battery
from drone_config import DroneConfig


class RotorPyVehicleAdapter(VehicleInterface):
    """
    Adapter wrapping RotorPy vehicle dynamics, controller, and environment.
    """

    def __init__(
        self,
        rotorpy_vehicle,
        rotorpy_environment,
        rotorpy_controller,
        config: DroneConfig,
        vehicle_id: str,
        home_position: Vector3 = Vector3(0, 0, 0),
        battery: Battery | None = None,
        wind: Wind | None = None,
    ):
        self.vehicle = rotorpy_vehicle
        self.environment = rotorpy_environment
        self.controller = rotorpy_controller
        self.config = config
        self._vehicle_id = vehicle_id
        self.home_position = home_position
        self.sim_time = 0.0
        self.wind = wind if wind is not None else Wind(velocity=Vector3(0, 0, 0))

        if battery is None:
            raise ValueError("RotorPyVehicleAdapter requires a Battery.")

        self.state = VehicleState(
            position=home_position,
            battery=battery,
            name=config.name,
            state=DroneState.LANDED,
            payload_capacity=config.payload_capacity,
            cruise_speed=config.cruise_speed,
        )

        self.current_route: Route | None = None
        self.current_waypoint_idx: int = 0

        # self.cruise_speed = config.cruise_speed

    @property
    def id(self) -> str:
        return self._vehicle_id

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
            self.state.velocity = Vector3(0, 0, 0)
            return

        # 1. Battery discharge during active flight
        self.state.battery.discharge(
            dt=dt,
            payload_weight=self.state.payload_weight,
            wind=self.wind,
            cruise_speed=self.config.cruise_speed,
        )

        if self.state.battery.is_empty():
            self.state.velocity = Vector3(0, 0, 0)
            self.state.state = DroneState.EMERGENCY
            return
        
        # 2. Check route completion to prevent drift / runaway physics integration
        if not self.current_route or self.current_waypoint_idx >= len(self.current_route.waypoints):
            self.state.state = DroneState.HOVERING
            return

        # 3. Target tracking logic
        target = self.current_route.waypoints[self.current_waypoint_idx].position
        dist = self.state.position.distance_to(target)
        step = self.config.cruise_speed * dt

        if dist <= step:
            # Reached waypoint
            self.state.position = target
            self.state.velocity = Vector3(0, 0, 0)
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

            self.state.velocity = direction * self.config.cruise_speed
            self.state.position = self.state.position + self.state.velocity * dt

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
        self.state.battery.reset()
        self.state.state = DroneState.LANDED
        self.state.payload_loaded = False
        self.state.payload_weight = 0.0
        self.current_route = None
        self.current_waypoint_idx = 0
        self.sim_time = 0.0

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.config.name,
            "type": self.config.name,
            "specs": {
                "cruise_speed": self.config.cruise_speed,
                "payload_capacity": self.config.payload_capacity,
                "rotor_speed": self.config.rotor_speed,
                "battery_capacity_wh": self.config.battery_capacity_wh,
            },
        }

    def sample(self, time: float) -> dict:
        return {
            "time": time,
            "position": {
                "x": self.state.position.x,
                "y": self.state.position.y,
                "z": self.state.position.z,
            },
            "velocity": {
                "x": self.state.velocity.x,
                "y": self.state.velocity.y,
                "z": self.state.velocity.z,
            },
            "battery": {
                "remaining_wh": self.state.battery.remaining_energy(),
                "capacity_wh": self.state.battery.capacity_wh,
                "remaining_percent": self.state.battery.remaining_percent(),
            },
            "payload_weight": self.state.payload_weight,
            "state": self.state.state.value,
            "fault": self.state.fault,
        }