# contracts.py
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class MissionState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    READY = "ready"
    RECHARGING = "recharging"
    EXECUTING = "executing"
    DELIVERING = "delivering"
    RETURNING = "returning"
    COMPLETE = "complete"
    ABORTED = "aborted"
    FAILED = "failed"


class DroneState(Enum):
    LANDED = "landed"
    FLYING = "flying"
    RETURNING = "returning"
    EMERGENCY = "emergency"
    HOVERING = "hovering"


@dataclass
class Vector3:
    x: float
    y: float
    z: float = 0.0

    def distance_to(self, other: "Vector3") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)


@dataclass
class Waypoint:
    position: Vector3
    altitude: Optional[float] = None
    speed: Optional[float] = None

    def __post_init__(self):
        if self.altitude is not None:
            self.position.z = self.altitude


@dataclass
class Route:
    waypoints: list[Waypoint] = field(default_factory=list)

    @property
    def distance(self) -> float:
        return sum(
            self.waypoints[i - 1].position.distance_to(self.waypoints[i].position)
            for i in range(1, len(self.waypoints))
        )

    def __len__(self):
        return len(self.waypoints)


@dataclass
class VehicleState:
    position: Vector3
    velocity: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))
    battery_percent: float = 100.0
    state: DroneState = DroneState.LANDED
    payload_loaded: bool = False
    connected: bool = True
    fault: Optional[str] = None
    payload_capacity: float = 5.0  # kg a single trip can carry; used for delivery batching


@dataclass
class Obstacle:
    position: Vector3
    width: float
    depth: float
    height: float

    def contains(self, point: Vector3, margin: float = 0.0) -> bool:
        return (
            abs(point.x - self.position.x) <= self.width / 2 + margin
            and abs(point.y - self.position.y) <= self.depth / 2 + margin
            and point.z <= self.height
        )


@dataclass
class Wind:
    velocity: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))

    @property
    def speed(self) -> float:
        return self.velocity.distance_to(Vector3(0, 0, 0))


# @dataclass
# class DeliveryPoint:
#     id: str
#     position: Vector3
#     priority: int = 0


# @dataclass
# class Scenario:
#     bounds: tuple[float, float, float, float]
#     obstacles: list[Obstacle] = field(default_factory=list)
#     wind: Wind = field(default_factory=Wind)
#     delivery_points: list[DeliveryPoint] = field(default_factory=list)
#     base: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))
#     drone_start_battery: float = 100.0
# contracts.py

@dataclass
class DeliveryOrder:
    id: str
    destination: Vector3
    priority: int = 0
    payload_weight: float = 1.0  # default weight for editor-placed orders
    status: str = "pending"      # "pending", "assigned", "in_progress", "delivered"


@dataclass
class Scenario:
    bounds: tuple[float, float, float, float]
    obstacles: list[Obstacle] = field(default_factory=list)
    wind: Wind = field(default_factory=Wind)
    orders: list[DeliveryOrder] = field(default_factory=list)  # Replaces delivery_points
    base: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))
    drone_start_battery: float = 100.0


@dataclass
class SimulationSnapshot:
    time: float
    vehicles: list[VehicleState]
    scenario: Scenario

    @property
    def vehicle(self) -> Optional[VehicleState]:
        """Convenience accessor for single-drone code/tests: the first vehicle."""
        return self.vehicles[0] if self.vehicles else None


# @dataclass
# class DeliveryOrder:
#     id: str
#     destination: Vector3
#     priority: int = 0
#     payload_weight: float = 0.0
#     status: str = "pending"


@dataclass
class DroneMission:
    """Tracks one drone's current assignment within a fleet."""
    id: str
    vehicle: "VehicleInterface"
    state: MissionState = MissionState.IDLE
    active_orders: list[DeliveryOrder] = field(default_factory=list)
    active_route: Optional[Route] = None


class VehicleInterface(ABC):

    @abstractmethod
    def get_state(self) -> VehicleState:
        ...

    @abstractmethod
    def upload_route(self, route: Route) -> None:
        ...

    @abstractmethod
    def start_mission(self) -> None:
        ...

    @abstractmethod
    def pause_mission(self) -> None:
        ...

    @abstractmethod
    def abort_mission(self) -> None:
        ...

    @abstractmethod
    def return_to_base(self) -> None:
        ...

    @abstractmethod
    def advance(self, dt: float) -> None:
        ...

    @abstractmethod
    def reset(self) -> None:
        ...


class EnvironmentInterface(ABC):

    @abstractmethod
    def get_scenario(self) -> Scenario:
        ...

    @abstractmethod
    def set_scenario(self, scenario: Scenario) -> None:
        ...

    @abstractmethod
    def get_obstacles(self) -> list[Obstacle]:
        ...

    @abstractmethod
    def get_wind(self) -> Wind:
        ...


class SimulationInterface(ABC):

    @abstractmethod
    def advance(self, dt: float) -> None:
        ...

    @abstractmethod
    def get_snapshot(self) -> SimulationSnapshot:
        ...

    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...