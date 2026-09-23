import os
import tempfile
from contracts import DeliveryOrder, Obstacle, Scenario, Vector3
from scenario import ScenarioManager


def test_scenario_manager_add_and_clear_orders(scenario_manager):
    order = DeliveryOrder("p1", Vector3(5, 5, 2), priority=1, payload_weight=1.5)
    scenario_manager.get_scenario().orders.append(order)
    
    assert len(scenario_manager.get_scenario().orders) == 1
    
    scenario_manager.get_scenario().orders.clear()
    assert len(scenario_manager.get_scenario().orders) == 0


def test_save_and_load_scenario():
    scenario = Scenario(
        bounds=(0, 0, 100, 100),
        base=Vector3(2, 2, 0),
        obstacles=[Obstacle(Vector3(10, 10, 0), 2, 2, 5)],
        orders=[DeliveryOrder("p1", Vector3(20, 20, 2), priority=2, payload_weight=1.5)],
    )
    env = ScenarioManager(scenario)

    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        env.save(tmp_path)
        loaded_env = ScenarioManager.load(tmp_path)
        loaded_sc = loaded_env.get_scenario()

        assert loaded_sc.base == Vector3(2, 2, 0)
        assert len(loaded_sc.obstacles) == 1
        assert len(loaded_sc.orders) == 1
        assert loaded_sc.orders[0].id == "p1"
        assert loaded_sc.orders[0].priority == 2
        assert loaded_sc.orders[0].payload_weight == 1.5
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)