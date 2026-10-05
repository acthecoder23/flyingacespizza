from __future__ import annotations

import math
import random

from contracts import DeliveryOrder, Scenario, Vector3

# class OrderSpawner:
#     def __init__(self, interval_seconds: float = 30.0, max_active_orders: int = 5):
#         self.interval = interval_seconds
#         self.max_active_orders = max_active_orders
#         self.timer = 0.0
#         self.counter = 100

#     # def _is_valid_location(self, pos: Vector3, scenario: Scenario, buffer: float = 2.0) -> bool:
#     #     """Checks if a point is inside or dangerously close to any obstacle."""
#     #     for obs in scenario.obstacles:
#     #         # Expand obstacle bounds by safety buffer
#     #         if (obs.position.x - obs.width/2 - buffer <= pos.x <= obs.position.x + obs.width/2 + buffer and
#     #             obs.position.y - obs.depth/2 - buffer <= pos.y <= obs.position.y + obs.depth/2 + buffer):
#     #             return False
#     #     return True
#     def _is_valid_location(self, pos: Vector3, scenario: Scenario, buffer: float = 2.0) -> bool:
#         """Return True if the delivery location is clear of all obstacles."""
#         return not any(
#             obstacle.contains_point(pos, margin=buffer)
#             for obstacle in scenario.obstacles
#         )

#     def update(self, dt: float, scenario: Scenario):
#         self.timer += dt
#         if self.timer >= self.interval and len(scenario.orders) < self.max_active_orders:
#             self.timer = 0.0
#             x0, y0, x1, y1 = scenario.bounds
#             margin = 3.0

#             # Try up to 20 random points to find a clear location
#             for _ in range(20):
#                 rand_x = random.uniform(x0 + margin, x1 - margin)
#                 rand_y = random.uniform(y0 + margin, y1 - margin)
#                 candidate_pos = Vector3(rand_x, rand_y, 2.0)

#                 if self._is_valid_location(candidate_pos, scenario):
#                     self.counter += 1
#                     new_order = DeliveryOrder(
#                         id=f"order-{self.counter}",
#                         destination=candidate_pos,
#                         priority=random.randint(1, 3),
#                         payload_weight=round(random.uniform(0.8, 2.0), 1),
#                     )
#                     scenario.orders.append(new_order)
#                     break

class OrderSpawner:
    def __init__(
        self,
        scenario: Scenario,
        spawn_interval: float = 5.0,
        max_active_orders: int = 10,
        landing_clearance: float = 2.0,
        search_radius: float = 100.0,
        search_step: float = 10.0,
    ):
        self.scenario = scenario
        self.interval = spawn_interval
        self.max_active_orders = max_active_orders

        self.landing_clearance = landing_clearance
        self.search_radius = search_radius
        self.search_step = search_step

        self.timer = 0.0
        self.next_order_id = 1

    def update(self, dt: float) -> None:
        self.timer += dt

        if self.timer < self.interval:
            return

        self.timer = 0.0

        if len(self.scenario.orders) >= self.max_active_orders:
            return

        spawn_point = self._random_spawn_point()

        if spawn_point is None:
            return

        destination = self._find_landing_spot(spawn_point)

        if destination is None:
            return

        order = DeliveryOrder(
            id=self.next_order_id,
            destination=destination,
            priority=random.randint(1, 3),
            payload_weight=round(random.uniform(0.5, 2.0), 1),
        )

        self.next_order_id += 1
        self.scenario.orders.append(order)

    def _random_spawn_point(self) -> Vector3 | None:
        min_x, min_y, max_x, max_y = self.scenario.bounds

        for _ in range(20):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)

            point = Vector3(x, y, 0.0)

            if self._within_bounds(point):
                return point

        return None

    def _find_landing_spot(self, origin: Vector3) -> Vector3 | None:
        if self._is_valid_location(origin):
            return Vector3(origin.x, origin.y, 0.0)

        radius = self.search_step

        while radius <= self.search_radius:
            candidates = self._search_ring(origin, radius)

            random.shuffle(candidates)

            for candidate in candidates:
                if self._is_valid_location(candidate):
                    return candidate

            radius += self.search_step

        return None

    def _search_ring(self, origin: Vector3, radius: float) -> list[Vector3]:
        circumference = 2.0 * math.pi * radius
        count = max(8, math.ceil(circumference / self.search_step))

        points = []

        for i in range(count):
            angle = 2.0 * math.pi * i / count

            points.append(
                Vector3(
                    origin.x + math.cos(angle) * radius,
                    origin.y + math.sin(angle) * radius,
                    0.0,
                )
            )

        return points

    def _is_valid_location(self, point: Vector3) -> bool:
        if not self._within_bounds(point):
            return False

        return not any(
            obstacle.contains_point(
                point,
                margin=self.landing_clearance,
            )
            for obstacle in self.scenario.obstacles
        )

    def _within_bounds(self, point: Vector3) -> bool:
        min_x, min_y, max_x, max_y = self.scenario.bounds

        return (
            min_x <= point.x <= max_x
            and min_y <= point.y <= max_y
        )