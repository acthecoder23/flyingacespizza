import pytest
from contracts import DeliveryOrder, Vector3, VehicleState
from mission_planner import MissionPlanner


def test_direct_clear_route(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    start = Vector3(0, 0, 2)
    dest = Vector3(5, 5, 2)
    
    route = planner.plan_route(start, dest)
    assert len(route.waypoints) == 2
    assert planner.route_is_clear(route) is True


def test_obstacle_avoidance_routing(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    # Direct path passes through obstacle at (20, 20)
    start = Vector3(10, 20, 2)
    dest = Vector3(30, 20, 2)
    
    route = planner.plan_route(start, dest)
    assert len(route.waypoints) > 2
    assert planner.route_is_clear(route) is True


def test_plan_delivery_batch_capacity_and_sorting(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    vehicle_state = VehicleState(position=Vector3(0, 0, 0), payload_capacity=3.0)
    
    # Safe locations away from obstacle at (20, 20)
    orders = [
        DeliveryOrder(id="low-priority", destination=Vector3(2, 2, 2), payload_weight=2.0, priority=1),
        DeliveryOrder(id="high-priority", destination=Vector3(3, 3, 2), payload_weight=2.0, priority=10),
    ]
    
    batch, route = planner.plan_delivery_batch(vehicle_state, orders, scenario_manager.get_scenario().base)
    
    assert len(batch) == 1
    assert batch[0].id == "high-priority"
    assert route is not None


def test_insufficient_battery_reserve_rejects_route(scenario_manager):
    planner = MissionPlanner(scenario_manager)
    # Vehicle has only 2% battery left
    vehicle_state = VehicleState(position=Vector3(0, 0, 0), battery_percent=2.0, payload_capacity=5.0)
    
    orders = [
        DeliveryOrder(id="far-order", destination=Vector3(40, 40, 2), payload_weight=1.0)
    ]
    
    batch, route = planner.plan_delivery_batch(vehicle_state, orders, scenario_manager.get_scenario().base)
    assert batch == []
    assert route is None