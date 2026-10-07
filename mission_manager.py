from contracts import *
from mission_planner import MissionPlanner


class MissionManager:
    """
    Manages a fleet of one or more drones. Each drone is tracked as a DroneMission
    (its own state/route/order-batch). Orders are shared across the fleet and
    assigned to whichever drone becomes available; plan_next_mission() uses the
    planner's delivery-batching to load a drone with as many deliveries as it can
    safely carry in one trip.
    """

    def __init__(
        self,
        vehicles,
        environment: EnvironmentInterface,
        planner: MissionPlanner,
    ):
        if isinstance(vehicles, VehicleInterface):
            vehicles = [vehicles]

        name_counts = {}
        self.missions = []
        self.completed_missions = []
        self.time = 0.0

        for vehicle in vehicles:
            name = vehicle.get_state().name
            name_counts[name] = name_counts.get(name, 0) + 1

            self.missions.append(
                DroneMission(
                    id=f"mission-{len(self.missions) + 1}",
                    vehicle=vehicle,
                    created_at=0.0,
                )
            )

        self.environment = environment
        self.planner = planner
        

    @property
    def vehicles(self) -> list[VehicleInterface]:
        return [m.vehicle for m in self.missions]

    @property
    def orders(self) -> list[DeliveryOrder]:
        """Dynamically exposes all orders from the active scenario."""
        return self.environment.get_scenario().orders

    @property
    def pending_orders(self) -> list[DeliveryOrder]:
        """Filter strictly for pending orders that haven't been dispatched yet."""
        return sorted(
            [o for o in self.orders if o.status == "pending"],
            key=lambda o: o.priority,
            reverse=True,
        )

    def add_order(self, order: DeliveryOrder):
        self.environment.get_scenario().orders.append(order)

    def _mission_by_id(self, mission_id: str | None) -> DroneMission | None:
        if mission_id is None:
            return None
        return next((m for m in self.missions if m.id == mission_id), None)

    def _first_idle_mission(self) -> DroneMission | None:
        return next(
            (m for m in self.missions if m.state in (MissionState.IDLE, MissionState.COMPLETE)),
            None,
        )

    def _first_in_state(self, state: MissionState) -> DroneMission | None:
        return next((m for m in self.missions if m.state == state), None)

    def plan_next_mission(self, mission_id: str | None = None) -> Route | None:
        mission = self._mission_by_id(mission_id) or self._first_idle_mission()
        if mission is None or not self.pending_orders:
            return None

        vehicle_state = mission.vehicle.get_state()
        scenario = self.environment.get_scenario()
        mission.state = MissionState.PLANNING

        batch, route = self.planner.plan_delivery_batch(
            vehicle_state, self.pending_orders, scenario.base
        )

        if not batch or route is None:
            # Revert to IDLE instead of persistent FAILED state so drone can attempt later
            mission.state = MissionState.IDLE
            return None

        for order in batch:
            order.status = "dispatched"

        mission.mission_number += 1
        mission.assigned_at = self.time
        mission.completed_at = None
        mission.failed_at = None
        mission.orders_fulfilled = 0
        mission.current_waypoint = 0
        
        mission.active_orders = batch
        mission.active_route = route
        mission.vehicle.upload_route(route)
        mission.state = MissionState.READY
        return route

    def start_next_mission(self, mission_id: str | None = None) -> bool:
        mission = self._mission_by_id(mission_id) or self._first_in_state(MissionState.READY)

        if mission is None:
            mission = self._mission_by_id(mission_id) or self._first_idle_mission()
            if mission is None:
                return False
            if self.plan_next_mission(mission.id) is None:
                return False

        mission.vehicle.start_mission()
        for order in mission.active_orders:
            order.status = "in_progress"
        mission.state = MissionState.EXECUTING
        return True

    def update(self, dt: float = 0.1, time: float | None = None):
        if time is not None:
            self.time = time

        scenario_base = self.environment.get_scenario().base

        # 1. Update active flight states and delivery progress.
        for mission in self.missions:
            self._update_mission(mission)

        # 2. Base/recharging logic.
        for mission in self.missions:
            v_state = mission.vehicle.get_state()
            at_base = v_state.position.distance_to(scenario_base) <= 2.5

            if mission.state in (MissionState.IDLE, MissionState.COMPLETE):
                if v_state.battery.remaining_percent() < 30.0 and at_base:
                    mission.state = MissionState.RECHARGING

            elif mission.state == MissionState.RECHARGING:
                if not at_base:
                    mission.state = MissionState.IDLE
                    continue

                v_state.battery.recharge(dt)

                if v_state.battery.is_full():
                    mission.state = MissionState.IDLE

        # 3. Automatically assign and start missions.
        self._auto_dispatch()


    def _auto_dispatch(self):
        if not self.pending_orders:
            return

        scenario_base = self.environment.get_scenario().base

        for mission in self.missions:
            if not self.pending_orders:
                break

            if mission.state not in (MissionState.IDLE, MissionState.COMPLETE):
                continue

            vehicle_state = mission.vehicle.get_state()

            at_base = (
                vehicle_state.position.distance_to(scenario_base) <= 2.5
            )

            if not at_base:
                continue

            if vehicle_state.battery.remaining_percent() < 30.0:
                continue

            route = self.plan_next_mission(mission.id)

            if route is None:
                continue

            if mission.state != MissionState.READY:
                continue

            self.start_next_mission(mission.id)

    def _update_mission(self, mission: DroneMission):
        state = mission.vehicle.get_state()

        mission.current_waypoint = getattr(mission.vehicle, "current_waypoint_idx", None)
            
        if state.fault:
            self.abort_mission(mission.id)
            return

        if mission.state in (MissionState.EXECUTING, MissionState.RETURNING):
            if state.battery.remaining_percent() < 15.0:
                self.abort_mission(mission.id)
                return

            # Check order drop-offs
            for order in mission.active_orders:
                if order.status in ("dispatched", "in_progress"):
                    dist = state.position.distance_to(order.destination)
                    if dist < 10.0 and order.status == "dispatched":
                        order.status = "in_progress"
                    if dist < 2.5 and order.status != "delivered":
                        order.status = "delivered"
                        order.completed_at = self.time
                        mission.orders_fulfilled += 1

            # Auto Return-to-Base after deliveries
            if mission.active_orders and all(o.status == "delivered" for o in mission.active_orders):
                scenario_base = self.environment.get_scenario().base
                if mission.state != MissionState.RETURNING:
                    self.return_to_base(mission.id)
                elif state.position.distance_to(scenario_base) < 2.5:
                    self.complete_mission(mission.id)

    def complete_mission(self, mission_id: str):
        mission = self._mission_by_id(mission_id)
        if not mission:
            return

        mission.completed_at = self.time
        mission.state = MissionState.COMPLETE
        self._archive_mission(mission)

        mission.active_orders = []
        mission.active_route = None

    def _archive_mission(self, mission: DroneMission):
        self.completed_missions.append(mission.to_dict())

    def abort_mission(self, mission_id: str | None = None):
        mission = self._mission_by_id(mission_id) or self.missions[0]
        if not mission:
            return

        mission.vehicle.abort_mission()
        mission.failed_at = self.time
        mission.state = MissionState.FAILED
        self._archive_mission(mission)

        for order in mission.active_orders:
            if order.status in ("dispatched", "in_progress"):
                order.status = "pending"

        mission.active_orders = []
        mission.active_route = None

    def return_to_base(self, mission_id: str | None = None):
        mission = self._mission_by_id(mission_id) or self._first_in_state(MissionState.EXECUTING)
        if mission is None:
            return

        mission.vehicle.return_to_base()
        mission.state = MissionState.RETURNING

    def reset(self):
        for mission in self.missions:
            mission.active_orders = []
            mission.active_route = None
            mission.state = MissionState.IDLE
            mission.assigned_at = None
            mission.completed_at = None
            mission.failed_at = None
            mission.orders_fulfilled = 0
            mission.current_waypoint = None
            mission.vehicle.reset()

        for order in self.orders:
            order.status = "pending"
            order.completed_at = None
            order.failed_at = None

    @property
    def state(self) -> MissionState:
        return self.missions[0].state if self.missions else MissionState.IDLE

    @property
    def active_route(self) -> Route | None:
        return self.missions[0].active_route if self.missions else None

    @property
    def active_order(self) -> DeliveryOrder | None:
        orders = self.missions[0].active_orders if self.missions else []
        return orders[0] if orders else None