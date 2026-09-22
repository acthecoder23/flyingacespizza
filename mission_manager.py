# mission_manager.py
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
        vehicles,  # VehicleInterface or list[VehicleInterface]
        environment: EnvironmentInterface,
        planner: MissionPlanner,
    ):
        if isinstance(vehicles, VehicleInterface):
            vehicles = [vehicles]

        self.missions: list[DroneMission] = [
            DroneMission(id=f"drone-{i + 1}", vehicle=v)
            for i, v in enumerate(vehicles)
        ]
        self.environment = environment
        self.planner = planner
        self.orders: list[DeliveryOrder] = []

    @property
    def vehicles(self) -> list[VehicleInterface]:
        return [m.vehicle for m in self.missions]

    def add_order(self, order: DeliveryOrder):
        self.orders.append(order)
        self.orders.sort(key=lambda x: -x.priority)

    # ------------------------------------------------------------------
    # Mission lookup helpers
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Mission control (mission_id optional: defaults to a sensible drone,
    # so single-fleet callers don't need to know about fleet management)
    # ------------------------------------------------------------------

    def plan_next_mission(self, mission_id: str | None = None) -> Route | None:
        mission = self._mission_by_id(mission_id) or self._first_idle_mission()
        if mission is None or not self.orders:
            return None

        vehicle_state = mission.vehicle.get_state()
        scenario = self.environment.get_scenario()
        mission.state = MissionState.PLANNING

        batch, route = self.planner.plan_delivery_batch(
            vehicle_state, self.orders, scenario.base
        )

        if not batch:
            mission.state = MissionState.FAILED
            return None

        for order in batch:
            self.orders.remove(order)
            order.status = "assigned"

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

    def update(self):
        for mission in self.missions:
            self._update_mission(mission)

    def _update_mission(self, mission: DroneMission):
        state = mission.vehicle.get_state()

        if state.fault:
            self.abort_mission(mission.id)
            return

        if mission.state == MissionState.EXECUTING:
            if state.battery_percent < 20:
                self.return_to_base(mission.id)
                return

            for order in mission.active_orders:
                if (
                    order.status == "in_progress"
                    and state.position.distance_to(order.destination) < 2.0
                ):
                    order.status = "delivered"

            if mission.active_orders and all(
                o.status == "delivered" for o in mission.active_orders
            ):
                mission.vehicle.return_to_base()
                mission.state = MissionState.RETURNING

        elif mission.state == MissionState.RETURNING:
            base = self.environment.get_scenario().base

            if (
                state.position.distance_to(base) < 0.5
                and state.velocity.distance_to(Vector3(0, 0, 0)) < 0.5
            ):
                self.complete_mission(mission.id)

    def complete_mission(self, mission_id: str | None = None):
        mission = self._mission_by_id(mission_id) or self._first_in_state(MissionState.RETURNING)
        if mission is None:
            return

        mission.active_orders = []
        mission.active_route = None
        mission.state = MissionState.COMPLETE

    def abort_mission(self, mission_id: str | None = None):
        mission = self._mission_by_id(mission_id) or self._first_in_state(MissionState.EXECUTING)
        if mission is None:
            return

        mission.vehicle.abort_mission()

        requeued = False
        for order in mission.active_orders:
            if order.status != "delivered":
                order.status = "pending"
                self.orders.append(order)
                requeued = True

        if requeued:
            self.orders.sort(key=lambda x: -x.priority)

        mission.active_orders = []
        mission.state = MissionState.ABORTED

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
        for order in self.orders:
            order.status = "pending"

    # ------------------------------------------------------------------
    # Backward-compatible single-drone accessors (mission[0])
    # ------------------------------------------------------------------

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
