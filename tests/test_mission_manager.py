from conftest import MockVehicle
from contracts import DeliveryOrder, MissionState, Vector3
from mission_manager import MissionManager
from mission_planner import MissionPlanner


def test_plan_and_start_mission(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    vehicle = MockVehicle()
    manager = MissionManager(vehicle, scenario_manager, planner)
    
    order = DeliveryOrder(id="order-1", destination=Vector3(5, 5, 2), priority=1, payload_weight=1.0)
    manager.add_order(order)

    route = manager.plan_next_mission()
    assert route is not None
    assert manager.state == MissionState.READY

    started = manager.start_next_mission()
    assert started is True
    assert manager.state == MissionState.EXECUTING


def test_recharge_triggers_only_at_base(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    
    v_base = MockVehicle(position=Vector3(0, 0, 0), battery=20.0)
    v_away = MockVehicle(position=Vector3(40, 40, 0), battery=20.0)
    
    manager = MissionManager([v_base, v_away], scenario_manager, planner)
    manager.update(dt=1.0)
    
    # Base vehicle recharges, airborne/away vehicle does not
    assert manager.missions[0].state == MissionState.RECHARGING
    assert manager.missions[1].state != MissionState.RECHARGING


def test_recharge_completion(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    v_base = MockVehicle(position=Vector3(0, 0, 0), battery=90.0)
    
    manager = MissionManager(v_base, scenario_manager, planner)
    manager.missions[0].state = MissionState.RECHARGING
    
    # 1 second of recharge @ 15%/s -> reaches 100%
    manager.update(dt=1.0)
    assert v_base.state.battery_percent == 100.0
    assert manager.missions[0].state == MissionState.IDLE


def test_abort_mission_requeues_orders(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    v = MockVehicle(position=Vector3(0, 0, 0), battery=100.0)
    
    order = DeliveryOrder(id="order-1", destination=Vector3(5, 5, 2), payload_weight=1.0)
    scenario_manager.get_scenario().orders.append(order)
    
    manager = MissionManager(v, scenario_manager, planner)
    manager.plan_next_mission()
    
    assert order.status == "dispatched"
    
    manager.abort_mission()
    assert order.status == "pending"
    assert manager.missions[0].state == MissionState.FAILED


def test_vehicle_fault_triggers_abort(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    v = MockVehicle(position=Vector3(0, 0, 0), battery=100.0)
    manager = MissionManager(v, scenario_manager, planner)
    
    order = DeliveryOrder(id="order-1", destination=Vector3(5, 5, 2), payload_weight=1.0)
    manager.add_order(order)
    manager.plan_next_mission()
    manager.start_next_mission()
    
    # Inject fault on vehicle
    v.state.fault = "MOTOR_FAILURE"
    manager.update(dt=0.1)
    
    assert manager.missions[0].state == MissionState.FAILED
    assert order.status == "pending"

def test_autodispatch_and_recharge_flow(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    v = MockVehicle(position=Vector3(0, 0, 0), battery=25.0) # Battery < 30%
    manager = MissionManager(v, scenario_manager, planner)
    
    # 1. Trigger initial update -> Should start RECHARGING because battery < 30%
    manager.update(dt=1.0)
    assert manager.missions[0].state == MissionState.RECHARGING
    assert v.state.battery_percent == 40.0
    
    # 2. Charge to 100%
    manager.update(dt=5.0)
    assert v.state.battery_percent == 100.0
    assert manager.missions[0].state == MissionState.IDLE
    
    # 3. Add order -> Next update tick should AUTO-PLAN and AUTO-START mission
    order = DeliveryOrder("auto-pizza", Vector3(2, 2, 2), payload_weight=1.0)
    scenario_manager.get_scenario().orders.append(order)
    
    manager.update(dt=0.1)
    assert manager.missions[0].state == MissionState.EXECUTING
    assert order.status == "in_progress"

def test_auto_dispatch_and_execution_flow(scenario_manager):
    """Verifies that pending orders auto-plan AND auto-start without manual key presses."""
    planner = MissionPlanner(scenario_manager)
    vehicle = MockVehicle(position=Vector3(0, 0, 0), battery=100.0)
    manager = MissionManager(vehicle, scenario_manager, planner)

    order = DeliveryOrder(id="auto-pizza", destination=Vector3(2, 2, 2), payload_weight=1.0)
    scenario_manager.get_scenario().orders.append(order)

    # Trigger single update tick
    manager.update(dt=0.1)

    # Verify mission automatically moved from IDLE -> READY -> EXECUTING
    assert manager.missions[0].state == MissionState.EXECUTING
    assert order.status == "in_progress"


def test_battery_recharge_and_auto_resume(scenario_manager):
    """Verifies that a low-battery drone recharges at base and auto-dispatches after reaching 100%."""
    planner = MissionPlanner(scenario_manager)
    vehicle = MockVehicle(position=Vector3(0, 0, 0), battery=20.0)
    manager = MissionManager(vehicle, scenario_manager, planner)

    # 1. Update tick triggers RECHARGING state
    manager.update(dt=1.0)
    assert manager.missions[0].state == MissionState.RECHARGING

    # 2. Advance time to fully recharge
    for _ in range(6):
        manager.update(dt=1.0)  # +15%/s

    assert vehicle.state.battery_percent == 100.0
    assert manager.missions[0].state == MissionState.IDLE

    # 3. Add order -> Next update tick should auto-dispatch
    order = DeliveryOrder(id="queued-pizza", destination=Vector3(2, 2, 2), payload_weight=1.0)
    scenario_manager.get_scenario().orders.append(order)
    manager.update(dt=0.1)

    assert manager.missions[0].state == MissionState.EXECUTING


def test_hover_on_route_completion(scenario_manager):
    """Verifies that reaching target waypoints stops movement and prevents position drift."""
    from autopilot_adapter import AutopilotAdapter
    from contracts import Route, Waypoint

    adapter = AutopilotAdapter(speed=10.0)
    adapter.upload_route(Route([Waypoint(Vector3(0, 0, 0)), Waypoint(Vector3(5, 0, 0))]))
    adapter.start_mission()

    # Step 1 second @ 10 m/s -> Overshoots 5m waypoint target
    adapter.advance(1.0)

    # Position locked at destination, state transitioned to LANDED/HOVER
    assert adapter.get_state().position == Vector3(5, 0, 0)
    assert adapter.get_state().state in (MissionState.HOVERING, MissionState.IDLE)