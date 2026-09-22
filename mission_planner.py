from __future__ import annotations

import heapq
import math

from contracts import *


class MissionPlanner:
    def __init__(self, environment: EnvironmentInterface):
        self.environment = environment
        self.obstacle_margin = 2.0

        # Simple energy model used for feasibility checks (not physical flight dynamics).
        self.energy_per_distance = 0.05      # base energy per meter
        self.weight_penalty_factor = 0.02    # extra energy per meter per kg carried
        self.wind_penalty_factor = 0.01      # extra energy per meter per m/s of wind
        self.battery_reserve_margin = 0.85   # only spend this fraction of battery on a plan

    def plan_route(self, start: Vector3, destination: Vector3) -> Route:
        if self.route_is_clear(Route([
            Waypoint(start),
            Waypoint(destination),
        ])):
            return Route([
                Waypoint(start),
                Waypoint(destination),
            ])

        return self.plan_around_obstacles(start, destination)

    def route_is_clear(self, route: Route) -> bool:
        for i in range(1, len(route.waypoints)):
            a = route.waypoints[i - 1].position
            b = route.waypoints[i].position

            if not self.segment_is_clear(a, b):
                return False

        return True

    def segment_is_clear(self, a: Vector3, b: Vector3) -> bool:
        for obstacle in self.environment.get_obstacles():
            if min(a.z, b.z) > obstacle.height:
                continue

            if self.segment_intersects_obstacle(a, b, obstacle):
                return False

        return True

    def segment_intersects_obstacle(
        self,
        a: Vector3,
        b: Vector3,
        obstacle: Obstacle,
    ) -> bool:
        margin = self.obstacle_margin

        xmin = obstacle.position.x - obstacle.width / 2 - margin
        xmax = obstacle.position.x + obstacle.width / 2 + margin
        ymin = obstacle.position.y - obstacle.depth / 2 - margin
        ymax = obstacle.position.y + obstacle.depth / 2 + margin

        dx = b.x - a.x
        dy = b.y - a.y

        t_min = 0.0
        t_max = 1.0

        for p, d, lower, upper in (
            (a.x, dx, xmin, xmax),
            (a.y, dy, ymin, ymax),
        ):
            if abs(d) < 1e-9:
                if p < lower or p > upper:
                    return False
                continue

            t1 = (lower - p) / d
            t2 = (upper - p) / d

            if t1 > t2:
                t1, t2 = t2, t1

            t_min = max(t_min, t1)
            t_max = min(t_max, t2)

            if t_min > t_max:
                return False

        return True

    def plan_around_obstacles(
        self,
        start: Vector3,
        destination: Vector3,
    ) -> Route:
        obstacles = self.environment.get_obstacles()

        # Only consider obstacles that could affect the route.
        relevant_obstacles = [
            obstacle
            for obstacle in obstacles
            if min(start.z, destination.z) <= obstacle.height
        ]

        # Try flying over the relevant obstacles.
        max_height = max(
            (obstacle.height for obstacle in relevant_obstacles),
            default=max(start.z, destination.z),
        )

        clearance_altitude = (
            max_height
            + self.obstacle_margin
        )

        altitude_route = Route([
            Waypoint(Vector3(start.x, start.y, start.z)),
            Waypoint(Vector3(start.x, start.y, clearance_altitude)),
            Waypoint(Vector3(destination.x, destination.y, clearance_altitude)),
            Waypoint(Vector3(
                destination.x,
                destination.y,
                destination.z,
            )),
        ])

        if self.route_is_clear(altitude_route):
            return altitude_route

        # Fall back to horizontal routing.
        nodes = [
            Vector3(start.x, start.y, start.z),
            Vector3(destination.x, destination.y, destination.z),
        ]

        for obstacle in relevant_obstacles:
            margin = self.obstacle_margin

            xmin = (
                obstacle.position.x
                - obstacle.width / 2
                - margin
            )
            xmax = (
                obstacle.position.x
                + obstacle.width / 2
                + margin
            )
            ymin = (
                obstacle.position.y
                - obstacle.depth / 2
                - margin
            )
            ymax = (
                obstacle.position.y
                + obstacle.depth / 2
                + margin
            )

            z = max(start.z, destination.z)

            nodes.extend([
                Vector3(xmin, ymin, z),
                Vector3(xmin, ymax, z),
                Vector3(xmax, ymin, z),
                Vector3(xmax, ymax, z),
            ])

        path = self._shortest_visible_path(nodes)

        if path is None:
            raise RuntimeError(
                "No collision-free route could be found."
            )

        return Route([
            Waypoint(nodes[index])
            for index in path
        ])
    
    def _shortest_visible_path(
        self,
        nodes: list[Vector3],
    ) -> list[int] | None:
        graph: dict[int, list[tuple[int, float]]] = {
            i: []
            for i in range(len(nodes))
        }

        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if not self.segment_is_clear(nodes[i], nodes[j]):
                    continue

                distance = nodes[i].distance_to(nodes[j])

                graph[i].append((j, distance))
                graph[j].append((i, distance))

        distances = {
            i: math.inf
            for i in range(len(nodes))
        }

        previous = {
            i: None
            for i in range(len(nodes))
        }

        distances[0] = 0.0

        queue = [(0.0, 0)]

        while queue:
            distance, current = heapq.heappop(queue)

            if distance > distances[current]:
                continue

            if current == 1:
                break

            for neighbor, edge_distance in graph[current]:
                new_distance = distance + edge_distance

                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = current
                    heapq.heappush(
                        queue,
                        (new_distance, neighbor),
                    )

        if math.isinf(distances[1]):
            return None

        path = []
        current = 1

        while current is not None:
            path.append(current)
            current = previous[current]

        path.reverse()

        return self._remove_redundant_waypoints(path, nodes)

    def _remove_redundant_waypoints(
        self,
        path: list[int],
        nodes: list[Vector3],
    ) -> list[int]:
        if len(path) <= 2:
            return path

        result = [path[0]]
        i = 0

        while i < len(path) - 1:
            furthest = i + 1

            for j in range(i + 2, len(path)):
                if self.segment_is_clear(
                    nodes[path[i]],
                    nodes[path[j]],
                ):
                    furthest = j

            result.append(path[furthest])
            i = furthest

        return result

    def leg_energy(self, distance: float, weight: float, wind: Wind) -> float:
        base = distance * self.energy_per_distance
        weight_penalty = distance * weight * self.weight_penalty_factor
        wind_penalty = wind.speed * distance * self.wind_penalty_factor
        return base + weight_penalty + wind_penalty

    def estimate_energy(
        self,
        route: Route,
        vehicle: VehicleState,
    ) -> float:
        """Single-route estimate assuming no payload (kept for backward compatibility)."""
        wind = self.environment.get_wind()
        return self.leg_energy(route.distance, 0.0, wind)

    def route_feasible(
        self,
        route: Route,
        vehicle: VehicleState,
    ) -> bool:
        energy_required = self.estimate_energy(
            route,
            vehicle,
        )

        return energy_required < vehicle.battery_percent * self.battery_reserve_margin

    # ------------------------------------------------------------------
    # Multi-delivery support
    # ------------------------------------------------------------------

    def plan_multi_stop_route(self, start: Vector3, stops: list[Vector3]) -> Route:
        """Chains obstacle-avoiding legs through an ordered list of stops."""
        waypoints: list[Waypoint] = [Waypoint(start)]
        position = start

        for stop in stops:
            leg = self.plan_route(position, stop)
            waypoints.extend(leg.waypoints[1:])  # skip duplicate junction point
            position = stop

        return Route(waypoints)

    def estimate_multi_stop_energy(
        self,
        start: Vector3,
        stops_with_weights: list[tuple[Vector3, float]],
    ) -> float:
        """
        Energy for start -> stop1 -> stop2 -> ... Each entry's weight is the payload
        still on board *while flying that leg* (i.e. already excluding anything
        dropped off on an earlier leg).
        """
        wind = self.environment.get_wind()
        position = start
        total = 0.0

        for stop, weight_on_leg in stops_with_weights:
            leg = self.plan_route(position, stop)
            total += self.leg_energy(leg.distance, weight_on_leg, wind)
            position = stop

        return total

    def plan_delivery_batch(
        self,
        vehicle_state: VehicleState,
        candidate_orders: list[DeliveryOrder],
        base: Vector3,
    ) -> tuple[list[DeliveryOrder], Optional[Route]]:
        """
        Greedily assembles the largest batch of deliveries a single drone can carry
        out in one trip: respects payload_capacity, and only accepts an order if the
        resulting route (through all batched stops and back to base) still fits
        within the battery-energy budget.

        This is a simple, explainable heuristic (priority + nearest-neighbor +
        incremental feasibility check), not an optimal solver -- a reasonable
        starting point for students to improve on (e.g. swap in a real VRP solver).
        """
        capacity = vehicle_state.payload_capacity
        remaining = sorted(candidate_orders, key=lambda o: -o.priority)

        batch: list[DeliveryOrder] = []
        total_weight = 0.0

        while remaining:
            position = batch[-1].destination if batch else vehicle_state.position

            feasible_by_weight = [
                o for o in remaining if total_weight + o.payload_weight <= capacity
            ]
            if not feasible_by_weight:
                break

            candidate = min(
                feasible_by_weight,
                key=lambda o: position.distance_to(o.destination),
            )

            trial_batch = batch + [candidate]
            stops_with_weights = self._stops_with_remaining_weight(trial_batch, base)
            energy = self.estimate_multi_stop_energy(
                vehicle_state.position, stops_with_weights
            )

            if energy > vehicle_state.battery_percent * self.battery_reserve_margin:
                remaining.remove(candidate)
                continue

            batch = trial_batch
            total_weight += candidate.payload_weight
            remaining.remove(candidate)

        if not batch:
            return [], None

        stops = [order.destination for order in batch] + [base]
        route = self.plan_multi_stop_route(vehicle_state.position, stops)
        return batch, route

    @staticmethod
    def _stops_with_remaining_weight(
        batch: list[DeliveryOrder],
        base: Vector3,
    ) -> list[tuple[Vector3, float]]:
        carried = sum(order.payload_weight for order in batch)
        stops = []

        for order in batch:
            stops.append((order.destination, carried))
            carried -= order.payload_weight

        stops.append((base, 0.0))
        return stops