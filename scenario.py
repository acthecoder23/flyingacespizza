# scenario.py
import json
from dataclasses import asdict

from contracts import *


class ScenarioManager(EnvironmentInterface):

    def __init__(self, scenario: Scenario):
        self.scenario = scenario

    def get_scenario(self) -> Scenario:
        return self.scenario

    def set_scenario(self, scenario: Scenario) -> None:
        self.scenario = scenario

    def get_obstacles(self) -> list[Obstacle]:
        return self.scenario.obstacles

    def get_wind(self) -> Wind:
        return self.scenario.wind

    def add_obstacle(self, obstacle: Obstacle):
        self.scenario.obstacles.append(obstacle)

    def remove_obstacle(self, obstacle: Obstacle):
        self.scenario.obstacles.remove(obstacle)

    def set_wind(self, wind: Wind):
        self.scenario.wind = wind

    def add_delivery_point(self, delivery: DeliveryPoint):
        self.scenario.delivery_points.append(delivery)

    def remove_delivery_point(self, delivery: DeliveryPoint):
        self.scenario.delivery_points.remove(delivery)

    # ------------------------------------------------------------------
    # Persistence: lets students save/share hand-built scenarios as JSON.
    # ------------------------------------------------------------------

    def to_json(self) -> str:
        return json.dumps(asdict(self.scenario), indent=2)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            f.write(self.to_json())

    @staticmethod
    def from_json(data: str) -> "ScenarioManager":
        raw = json.loads(data)
        scenario = Scenario(
            bounds=tuple(raw["bounds"]),
            obstacles=[
                Obstacle(Vector3(**o["position"]), o["width"], o["depth"], o["height"])
                for o in raw["obstacles"]
            ],
            wind=Wind(Vector3(**raw["wind"]["velocity"])),
            delivery_points=[
                DeliveryPoint(d["id"], Vector3(**d["position"]), d["priority"])
                for d in raw["delivery_points"]
            ],
            base=Vector3(**raw["base"]),
            drone_start_battery=raw["drone_start_battery"],
        )
        return ScenarioManager(scenario)

    @staticmethod
    def load(path: str) -> "ScenarioManager":
        with open(path) as f:
            return ScenarioManager.from_json(f.read())