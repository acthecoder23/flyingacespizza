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
        # self.orders: list[DeliveryOrder] = []

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
            key=lambda x: -x.priority
        )
    
    def add_order(self, order: DeliveryOrder):
        self.environment.get_scenario().orders.append(order)

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
        if mission is None or not self.pending_orders:
            return None

        vehicle_state = mission.vehicle.get_state()
        scenario = self.environment.get_scenario()
        mission.state = MissionState.PLANNING

        batch, route = self.planner.plan_delivery_batch(
            vehicle_state, self.pending_orders, scenario.base
        )

        if not batch:
            mission.state = MissionState.FAILED
            return None

        # Mark all batch orders as DISPATCHED so no other drone targets them
        for order in batch:
            order.status = "dispatched"

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
        # 1. Process active missions
        for mission in self.missions:
            self._update_mission(mission)

        # 2. Recharging logic for idle/base drones
        for mission in self.missions:
            state = mission.vehicle.get_state()
            if mission.state in (MissionState.IDLE, MissionState.COMPLETE):
                if state.battery_percent < 30.0:  # Threshold to trigger recharge
                    mission.state = MissionState.RECHARGING

            elif mission.state == MissionState.RECHARGING:
                # Simulating recharge rate at base
                state.battery_percent = min(100.0, state.battery_percent + 15.0 * 0.1)
                if state.battery_percent >= 100.0:
                    mission.state = MissionState.IDLE

        # 3. Continuous Auto-Dispatch: assign pending orders to healthy idle drones
        if self.pending_orders:
            for mission in self.missions:
                if mission.state == MissionState.IDLE:
                    self.plan_next_mission(mission.id)
                    if mission.state == MissionState.READY:
                        self.start_next_mission(mission.id)

    def _update_mission(self, mission: DroneMission):
        state = mission.vehicle.get_state()

        if state.fault:
            self.abort_mission(mission.id)
            return

        if mission.state == MissionState.EXECUTING:
            # Check battery emergency threshold
            if state.battery_percent < 15:
                self.abort_mission(mission.id)
                return

            # Check drop-offs for orders in the active batch
            for order in mission.active_orders:
                if order.status in ("dispatched", "in_progress"):
                    dist = state.position.distance_to(order.destination)
                    
                    # Mark in-progress when drone is closing in
                    if dist < 10.0 and order.status == "dispatched":
                        order.status = "in_progress"

                    # Mark delivered when within drop range
                    if dist < 2.5:
                        order.status = "delivered"

            # Check if all orders in batch are delivered
            if mission.active_orders and all(o.status == "delivered" for o in mission.active_orders):
                scenario_base = self.environment.get_scenario().base
                if state.position.distance_to(scenario_base) < 2.0:
                    self.complete_mission(mission.id)

    def complete_mission(self, mission_id: str):
        """Clean up completed orders and reset route on mission completion."""
        mission = self._mission_by_id(mission_id)
        if not mission:
            return

        scenario = self.environment.get_scenario()
        
        # Remove completed orders permanently from the map/scenario
        scenario.orders = [
            o for o in scenario.orders if o.status != "delivered"
        ]

        mission.active_orders = []
        mission.active_route = None  # Removes flight path from map
        mission.state = MissionState.COMPLETE

    def abort_mission(self, mission_id: str):
        """Aborts mission, purges delivered parts, and requeues remaining orders."""
        mission = self._mission_by_id(mission_id)
        if not mission:
            return

        scenario = self.environment.get_scenario()

        # Remove orders that were successfully delivered before the abort happened
        scenario.orders = [
            o for o in scenario.orders if o.status != "delivered"
        ]

        mission.vehicle.abort_mission()
        mission.state = MissionState.FAILED

        # Requeue remaining undelivered orders back to 'pending'
        for order in mission.active_orders:
            if order.status in ("dispatched", "in_progress"):
                order.status = "pending"

        mission.active_orders = []
        mission.active_route = None  # Removes aborted route path from map

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
