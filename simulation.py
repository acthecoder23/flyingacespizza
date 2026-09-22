# simulation.py
from contracts import *


class SimulationManager(SimulationInterface):

    def __init__(
        self,
        vehicles,  # VehicleInterface or list[VehicleInterface]
        environment: EnvironmentInterface,
    ):
        if isinstance(vehicles, VehicleInterface):
            vehicles = [vehicles]

        self.vehicles: list[VehicleInterface] = list(vehicles)
        self.environment = environment
        self.time = 0.0
        self.running = False

    def advance(self, dt: float) -> None:
        if not self.running:
            return

        for vehicle in self.vehicles:
            vehicle.advance(dt)

        self.time += dt

    def get_snapshot(self) -> SimulationSnapshot:
        return SimulationSnapshot(
            time=self.time,
            vehicles=[vehicle.get_state() for vehicle in self.vehicles],
            scenario=self.environment.get_scenario(),
        )

    def reset(self) -> None:
        self.time = 0.0
        for vehicle in self.vehicles:
            vehicle.reset()

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False
