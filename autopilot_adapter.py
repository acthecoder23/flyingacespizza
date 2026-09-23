from contracts import VehicleInterface, VehicleState, DroneState, Route, Vector3

class AutopilotAdapter(VehicleInterface):
    def __init__(self, speed: float = 5.0):
        self.state = VehicleState(
            position=Vector3(0, 0, 0),
            battery_percent=100.0,
            state=DroneState.LANDED,
            payload_capacity=5.0
        )
        self.current_route: Route | None = None
        self.current_waypoint_idx: int = 0
        self.speed = speed

    def set_battery(self, battery_percent: float):
        """Allows external systems (e.g., base charger) to recharge battery."""
        self.state.battery_percent = max(0.0, min(100.0, battery_percent))

    def upload_route(self, route: Route) -> None:
        self.current_route = route
        self.current_waypoint_idx = 0

    def start_mission(self) -> None:
        self.state.state = DroneState.FLYING

    def advance(self, dt: float) -> None:
        if self.state.state not in (DroneState.FLYING, DroneState.RETURNING):
            return

        # Drain battery while flying
        self.state.battery_percent -= 0.5 * dt
        if self.state.battery_percent <= 0:
            self.state.battery_percent = 0
            self.state.state = DroneState.EMERGENCY
            return

        if not self.current_route or self.current_waypoint_idx >= len(self.current_route.waypoints):
            # Target reached: STOP DRIFTING, set velocity to zero and hover/land
            self.state.state = DroneState.HOVERING
            return

        target = self.current_route.waypoints[self.current_waypoint_idx].position
        dist = self.state.position.distance_to(target)
        step = self.speed * dt

        if dist <= step:
            self.state.position = target
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_route.waypoints):
                self.state.state = DroneState.LANDED
        else:
            dir_vec = Vector3(
                (target.x - self.state.position.x) / dist,
                (target.y - self.state.position.y) / dist,
                (target.z - self.state.position.z) / dist,
            )
            self.state.position = self.state.position + dir_vec * step