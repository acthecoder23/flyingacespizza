import numpy as np

from contracts import *
from scenario import ScenarioManager
from mission_planner import MissionPlanner
from mission_manager import MissionManager
from simulation import SimulationManager
from rotorpy_adapter import RotorPyVehicleAdapter
from pygame_ui import PygameUI

from rotorpy.environments import Environment
from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.controllers.quadrotor_control import SE3Control
from rotorpy.trajectories.hover_traj import HoverTraj
from rotorpy.wind.default_winds import ConstantWind

# Number of drones in the fleet. Each gets its own RotorPy vehicle/controller/
# environment instance and a small offset from base so they don't overlap at start.
FLEET_SIZE = 2

PAYLOAD_CAPACITY = 3.0  # kg a single drone can carry per trip


def build_vehicle(home_position: Vector3, wind: Wind) -> RotorPyVehicleAdapter:
    initial_state = {
        "x": np.array([home_position.x, home_position.y, home_position.z]),
        "v": np.zeros(3),
        "q": np.array([0.0, 0.0, 0.0, 1.0]),
        "w": np.zeros(3),
        "wind": np.zeros(3),
        "rotor_speeds": np.array([1788.53, 1788.53, 1788.53, 1788.53]),
    }

    rotorpy_vehicle = Multirotor(quad_params, initial_state=initial_state)
    rotorpy_controller = SE3Control(quad_params)

    rotorpy_environment = Environment(
        vehicle=rotorpy_vehicle,
        controller=rotorpy_controller,
        trajectory=HoverTraj(x0=np.array([home_position.x, home_position.y, home_position.z])),
        wind_profile=ConstantWind(wx=wind.velocity.x, wy=wind.velocity.y, wz=wind.velocity.z),
        sim_rate=100,
    )

    return RotorPyVehicleAdapter(
        rotorpy_vehicle=rotorpy_vehicle,
        rotorpy_environment=rotorpy_environment,
        rotorpy_controller=rotorpy_controller,
        home_position=home_position,
        payload_capacity=PAYLOAD_CAPACITY,
    )


def build_application():

    base = Vector3(5, 5, 2)

    scenario = Scenario(
        bounds=(0, 0, 40, 30),
        base=base,
        wind=Wind(Vector3(2.0, 0.0, 0.0)),
        obstacles=[
            Obstacle(Vector3(15, 10, 0), 4, 4, 5),
            Obstacle(Vector3(25, 20, 0), 3, 6, 4),
        ],
        delivery_points=[
            DeliveryPoint("pizza-1", Vector3(35, 25, 2)),
            DeliveryPoint("pizza-2", Vector3(32, 8, 2)),
            DeliveryPoint("pizza-3", Vector3(20, 27, 2)),
        ],
    )

    environment = ScenarioManager(scenario)

    vehicles = [
        build_vehicle(Vector3(base.x + i * 1.5, base.y, base.z), scenario.wind)
        for i in range(FLEET_SIZE)
    ]

    planner = MissionPlanner(environment)

    mission_manager = MissionManager(
        vehicles=vehicles,
        environment=environment,
        planner=planner,
    )

    # A handful of orders with different weights/priorities so the batching
    # heuristic has something interesting to pack.
    mission_manager.add_order(DeliveryOrder(id="order-001", destination=Vector3(35, 25, 2), priority=1, payload_weight=1.2))
    mission_manager.add_order(DeliveryOrder(id="order-002", destination=Vector3(32, 8, 2), priority=1, payload_weight=1.0))
    mission_manager.add_order(DeliveryOrder(id="order-003", destination=Vector3(20, 27, 2), priority=2, payload_weight=1.5))

    simulation = SimulationManager(
        vehicles=vehicles,
        environment=environment,
    )

    ui = PygameUI(
        scenario_manager=environment,
        mission_manager=mission_manager,
        simulation=simulation,
    )

    return ui


if __name__ == "__main__":
    app = build_application()
    app.run()
