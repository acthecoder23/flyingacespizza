from __future__ import annotations

import numpy as np

from contracts import *

class BatteryModel:
    def __init__(
        self,
        capacity=100.0,
        hover_rotor_speed=1788.53,
        nominal_power=1.0,
    ):
        self.capacity = capacity
        self.energy = capacity
        self.hover_rotor_speed = hover_rotor_speed
        self.nominal_power = nominal_power

    @property
    def percent(self):
        return self.energy / self.capacity * 100.0

    def update(self, rotor_speeds, dt):
        rotor_speeds = np.asarray(rotor_speeds)

        ratios = rotor_speeds / self.hover_rotor_speed

        power_factor = np.mean(ratios ** 3)

        energy_used = (
            self.nominal_power
            * power_factor
            * dt
        )

        self.energy = max(
            0.0,
            self.energy - energy_used,
        )

class RotorPyVehicleAdapter(VehicleInterface):

    def __init__(
        self,
        rotorpy_vehicle=None,
        rotorpy_environment=None,
        rotorpy_controller=None,
        home_position=None,
        battery_capacity=100.0,
        payload_capacity=5.0,
    ):
        self.rotorpy_vehicle = rotorpy_vehicle
        self.rotorpy_environment = rotorpy_environment
        self.rotorpy_controller = rotorpy_controller

        self.home_position = home_position or Vector3(0, 0, 1)
        self.payload_capacity = payload_capacity

        self.battery_capacity = battery_capacity
        self._battery = battery_capacity
        self.battery = BatteryModel()
        
        self._route = None
        self._trajectory = None
        self._time = 0.0
        self._rotorpy_state = None

        self._placeholder_route_index = 1

        self._sim_dt = 0.01
        self._dt_accumulator = 0.0

        self._state = VehicleState(
            position=Vector3(
                self.home_position.x,
                self.home_position.y,
                self.home_position.z,
            ),
            battery_percent=100.0,
            state=DroneState.LANDED,
            connected=rotorpy_vehicle is not None,
            payload_capacity=self.payload_capacity,
        )

    # ------------------------------------------------------------------
    # VehicleInterface
    # ------------------------------------------------------------------

    def get_state(self) -> VehicleState:
        return self._state

    def upload_route(self, route: Route) -> None:
        if len(route) < 1:
            raise ValueError("Cannot upload an empty route.")

        self._route = route
        self._trajectory = None
        self._placeholder_route_index = 1

    def start_mission(self) -> None:
        if self._route is None:
            raise RuntimeError("No route has been uploaded.")

        if self.rotorpy_vehicle is None:
            self._state.state = DroneState.FLYING
            self._placeholder_route_index = 1
            return

        if self.rotorpy_controller is None:
            raise RuntimeError("RotorPy controller is not configured.")

        self._initialize_rotorpy_state()
        self._trajectory = self._build_trajectory(self._route)

        self._time = 0.0
        self._state.state = DroneState.FLYING
        self._state.fault = None

    def pause_mission(self) -> None:
        if self._state.state == DroneState.FLYING:
            self._state.state = DroneState.LANDED

    def abort_mission(self) -> None:
        self._state.state = DroneState.EMERGENCY

    def return_to_base(self) -> None:
        if self._battery <= 0.0:
            self._state.state = DroneState.EMERGENCY
            self._state.fault = "Battery depleted"
            return

        current = self._state.position

        route = Route([
            Waypoint(Vector3(
                current.x,
                current.y,
                current.z,
            )),
            Waypoint(Vector3(
                self.home_position.x,
                self.home_position.y,
                self.home_position.z,
            )),
        ])

        self.upload_route(route)
        self.start_mission()
        self._state.state = DroneState.RETURNING

    def advance(self, dt: float) -> None:
        if dt <= 0:
            return

        if self._state.state not in (
            DroneState.FLYING,
            DroneState.RETURNING,
        ):
            return

        self._dt_accumulator += dt

        while self._dt_accumulator >= self._sim_dt:
            if self.rotorpy_vehicle is None:
                self._advance_placeholder(self._sim_dt)
            else:
                self._advance_rotorpy(self._sim_dt)

            self._dt_accumulator -= self._sim_dt

    def reset(self) -> None:
        self._route = None
        self._trajectory = None
        self._time = 0.0
        self._rotorpy_state = None
        self._placeholder_route_index = 1
        self._battery = self.battery_capacity

        self._state = VehicleState(
            position=Vector3(
                self.home_position.x,
                self.home_position.y,
                self.home_position.z,
            ),
            battery_percent=100.0,
            state=DroneState.LANDED,
            connected=self.rotorpy_vehicle is not None,
            payload_capacity=self.payload_capacity,
        )

    # ------------------------------------------------------------------
    # RotorPy
    # ------------------------------------------------------------------

    def _initialize_rotorpy_state(self) -> None:
        position = self._state.position

        wind = np.zeros(3)

        if self.rotorpy_environment is not None:
            wind_profile = getattr(
                self.rotorpy_environment,
                "wind_profile",
                None,
            )

            if wind_profile is not None:
                try:
                    wind = np.asarray(
                        wind_profile.update(0.0),
                        dtype=float,
                    )
                except (AttributeError, TypeError):
                    pass

        self._rotorpy_state = {
            "x": np.array([
                position.x,
                position.y,
                position.z,
            ], dtype=float),

            "v": np.zeros(3),

            "q": np.array([
                0.0,
                0.0,
                0.0,
                1.0,
            ]),

            "w": np.zeros(3),

            "wind": wind,

            "rotor_speeds": np.array([
                1788.53,
                1788.53,
                1788.53,
                1788.53,
            ]),
        }

    def _build_trajectory(self, route: Route):
        from rotorpy.trajectories.minsnap import MinSnap

        points = np.asarray(
            [
                [
                    waypoint.position.x,
                    waypoint.position.y,
                    waypoint.position.z,
                ]
                for waypoint in route.waypoints
            ],
            dtype=float,
        )

        if len(points) == 1:
            from rotorpy.trajectories.hover_traj import HoverTraj

            return HoverTraj(x0=points[0])

        return MinSnap(
            points,
            v_max=3.0,
            v_avg=1.0,
            verbose=False,
        )

    def _advance_rotorpy(self, dt: float) -> None:
        if self._rotorpy_state is None:
            raise RuntimeError("RotorPy state has not been initialized.")

        if self._trajectory is None:
            raise RuntimeError("RotorPy trajectory has not been initialized.")

        flat_output = self._trajectory.update(self._time)

        control = self.rotorpy_controller.update(
            self._time,
            self._rotorpy_state,
            flat_output,
        )

        self._rotorpy_state = self.rotorpy_vehicle.step(
            self._rotorpy_state,
            control,
            dt,
        )

        self._time += dt

        self._update_application_state()

        destination = self._route.waypoints[-1].position

        position_error = self._state.position.distance_to(destination)
        speed = self._state.velocity.distance_to(Vector3(0, 0, 0))
        keyframes = getattr(self._trajectory, "t_keyframes", None)
        trajectory_finished = (
            keyframes is None
            or self._time >= float(keyframes[-1])
        )

        if trajectory_finished and position_error < 0.5 and speed < 0.5:
            self._state.position = Vector3(
                destination.x,
                destination.y,
                destination.z,
            )
            self._state.velocity = Vector3(0, 0, 0)
            self._state.state = DroneState.LANDED

    def _update_application_state(self) -> None:
        state = self._rotorpy_state

        position = state["x"]
        velocity = state["v"]

        self._state.position = Vector3(
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )

        self._state.velocity = Vector3(
            float(velocity[0]),
            float(velocity[1]),
            float(velocity[2]),
        )

        self.battery.update(
            state["rotor_speeds"],
            self._sim_dt,
        )

        self._state.battery_percent = self.battery.percent

        if self._state.battery_percent <= 0.0:
            self._state.state = DroneState.EMERGENCY
            self._state.fault = "Battery depleted"
    # ------------------------------------------------------------------
    # Placeholder backend
    # ------------------------------------------------------------------

    def _advance_placeholder(self, dt: float) -> None:
        if self._route is None:
            return

        if self._placeholder_route_index >= len(self._route.waypoints):
            self._state.state = DroneState.LANDED
            return

        target = self._route.waypoints[
            self._placeholder_route_index
        ].position

        current = self._state.position

        direction = np.array([
            target.x - current.x,
            target.y - current.y,
            target.z - current.z,
        ])

        distance = np.linalg.norm(direction)

        if distance < 0.1:
            self._state.position = Vector3(
                target.x,
                target.y,
                target.z,
            )

            self._placeholder_route_index += 1

            if self._placeholder_route_index >= len(
                self._route.waypoints
            ):
                self._state.state = DroneState.LANDED

            return

        speed = 10.0
        travel = min(speed * dt, distance)

        direction /= distance

        self._state.position.x += direction[0] * travel
        self._state.position.y += direction[1] * travel
        self._state.position.z += direction[2] * travel

        self._state.velocity = Vector3(
            direction[0] * speed,
            direction[1] * speed,
            direction[2] * speed,
        )