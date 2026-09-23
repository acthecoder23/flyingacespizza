# router.py
import heapq
import math
from typing import List, Optional
from contracts import Vector3, DeliveryOrder, Route, Waypoint


class Router:
    def __init__(self, grid_size: float = 1.0, safety_margin: float = 2.0, min_cruise_altitude: float = 10.0):
        self.grid_size = grid_size
        self.safety_margin = safety_margin
        self.min_cruise_altitude = min_cruise_altitude

    def _get_required_altitude(self, obstacles) -> float:
        """Dynamically set cruise altitude higher than the tallest obstacle."""
        if not obstacles:
            return self.min_cruise_altitude
        max_h = max(obs.height for obs in obstacles)
        return max(self.min_cruise_altitude, max_h + 3.0)

    def _is_obstacle_2d(self, x: float, y: float, obstacles) -> bool:
        """Check 2D grid cell collision against obstacle bounds."""
        for obs in obstacles:
            min_x = obs.position.x - (obs.width / 2.0) - self.safety_margin
            max_x = obs.position.x + (obs.width / 2.0) + self.safety_margin
            min_y = obs.position.y - (obs.depth / 2.0) - self.safety_margin
            max_y = obs.position.y + (obs.depth / 2.0) + self.safety_margin

            if min_x <= x <= max_x and min_y <= y <= max_y:
                return True
        return False

    def find_2d_path(self, start_pos: Vector3, goal_pos: Vector3, obstacles, bounds, cruise_altitude: float) -> Optional[List[Vector3]]:
        bx_min, by_min, bx_max, by_max = bounds

        start_gx, start_gy = int(round(start_pos.x / self.grid_size)), int(round(start_pos.y / self.grid_size))
        goal_gx, goal_gy = int(round(goal_pos.x / self.grid_size)), int(round(goal_pos.y / self.grid_size))

        open_set = [(0.0, start_gx, start_gy)]
        came_from = {}
        g_score = {(start_gx, start_gy): 0.0}

        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]

        found = False
        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (goal_gx, goal_gy):
                found = True
                break

            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                wx, wy = nx * self.grid_size, ny * self.grid_size

                if not (bx_min <= wx <= bx_max and by_min <= wy <= by_max):
                    continue

                # Clear start & goal cells to prevent start-point lockouts
                if (nx, ny) != (start_gx, start_gy) and (nx, ny) != (goal_gx, goal_gy):
                    if self._is_obstacle_2d(wx, wy, obstacles):
                        continue

                tentative_g = g_score[(cx, cy)] + cost
                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    h = math.hypot(goal_gx - nx, goal_gy - ny)
                    heapq.heappush(open_set, (tentative_g + h, nx, ny))

        if not found:
            return None

        # Path reconstruction
        path = []
        curr = (goal_gx, goal_gy)
        while curr in came_from:
            wx, wy = curr[0] * self.grid_size, curr[1] * self.grid_size
            path.append(Vector3(wx, wy, cruise_altitude))
            curr = came_from[curr]

        path.append(Vector3(start_gx * self.grid_size, start_gy * self.grid_size, cruise_altitude))
        path.reverse()
        return path

    def plan_delivery_route(self, start_pos: Vector3, orders: List[DeliveryOrder], scenario) -> Optional[Route]:
        if not orders:
            return None

        cruise_altitude = self._get_required_altitude(scenario.obstacles)
        waypoints = []

        # Start Ascent
        waypoints.append(Waypoint(position=Vector3(start_pos.x, start_pos.y, cruise_altitude)))

        current_pos = Vector3(start_pos.x, start_pos.y, cruise_altitude)

        for order in orders:
            leg = self.find_2d_path(current_pos, order.destination, scenario.obstacles, scenario.bounds, cruise_altitude)
            if not leg:
                return None

            for pt in leg:
                waypoints.append(Waypoint(position=pt))

            # Delivery descent/ascent steps
            waypoints.append(Waypoint(position=Vector3(order.destination.x, order.destination.y, order.destination.z)))
            waypoints.append(Waypoint(position=Vector3(order.destination.x, order.destination.y, cruise_altitude)))
            current_pos = Vector3(order.destination.x, order.destination.y, cruise_altitude)

        # Base Return leg
        return_leg = self.find_2d_path(current_pos, scenario.base, scenario.obstacles, scenario.bounds, cruise_altitude)
        if not return_leg:
            return None

        for pt in return_leg:
            waypoints.append(Waypoint(position=pt))

        # Final Landing
        waypoints.append(Waypoint(position=Vector3(scenario.base.x, scenario.base.y, scenario.base.z)))

        return Route(waypoints=waypoints)