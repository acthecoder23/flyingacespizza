# order_spawner.py

import random
from contracts import DeliveryOrder, Vector3, Scenario

class OrderSpawner:
    def __init__(self, interval_seconds: float = 30.0, max_active_orders: int = 5):
        self.interval = interval_seconds
        self.max_active_orders = max_active_orders
        self.timer = 0.0
        self.counter = 100

    def _is_valid_location(self, pos: Vector3, scenario: Scenario, buffer: float = 2.0) -> bool:
        """Checks if a point is inside or dangerously close to any obstacle."""
        for obs in scenario.obstacles:
            # Expand obstacle bounds by safety buffer
            if (obs.position.x - obs.width/2 - buffer <= pos.x <= obs.position.x + obs.width/2 + buffer and
                obs.position.y - obs.depth/2 - buffer <= pos.y <= obs.position.y + obs.depth/2 + buffer):
                return False
        return True

    def update(self, dt: float, scenario: Scenario):
        self.timer += dt
        if self.timer >= self.interval and len(scenario.orders) < self.max_active_orders:
            self.timer = 0.0
            x0, y0, x1, y1 = scenario.bounds
            margin = 3.0

            # Try up to 20 random points to find a clear location
            for _ in range(20):
                rand_x = random.uniform(x0 + margin, x1 - margin)
                rand_y = random.uniform(y0 + margin, y1 - margin)
                candidate_pos = Vector3(rand_x, rand_y, 2.0)

                if self._is_valid_location(candidate_pos, scenario):
                    self.counter += 1
                    new_order = DeliveryOrder(
                        id=f"order-{self.counter}",
                        destination=candidate_pos,
                        priority=random.randint(1, 3),
                        payload_weight=round(random.uniform(0.8, 2.0), 1),
                    )
                    scenario.orders.append(new_order)
                    break