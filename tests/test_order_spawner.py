from contracts import DeliveryOrder
from order_spawner import OrderSpawner


def test_order_spawner_generation(scenario_manager):
    spawner = OrderSpawner(interval_seconds=0.5, max_active_orders=5)
    scenario = scenario_manager.get_scenario()
    
    # Advance time by 0.6 seconds to trigger first order spawn
    spawner.update(0.6, scenario)
    
    assert len(scenario.orders) == 1
    spawned_order = scenario.orders[0]
    
    # Verify generated order characteristics
    assert isinstance(spawned_order, DeliveryOrder)
    assert spawned_order.status == "pending"
    assert 0 <= spawned_order.destination.x <= scenario.bounds[2]
    assert 0 <= spawned_order.destination.y <= scenario.bounds[3]


def test_order_spawner_max_limit(scenario_manager):
    max_orders = 2
    spawner = OrderSpawner(interval_seconds=0.1, max_active_orders=max_orders)
    scenario = scenario_manager.get_scenario()
    
    # Advance enough time to trigger multiple spawns
    for _ in range(10):
        spawner.update(0.2, scenario)
        
    assert len(scenario.orders) == max_orders