from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from contracts import *
from battery import (
    Battery,
    BatteryDischargeModel,
    BatteryRechargeModel,
    ConstantPowerDischarge,
    ConstantRecharge,
    PayloadWindDischarge,
)
from drone_config import DroneConfig

from map_module import MapData
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


# ---------------------------------------------------------------------------
# Fleet configuration
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class DroneSpec:
    """
    Configuration for one drone type.

    This is intentionally lightweight for now. It can eventually become
    DroneConfig once the fleet architecture is settled.
    """

    name: str
    cruise_speed: float
    payload_capacity: float
    battery_capacity_wh: float

    discharge_model_factory: Callable[[], BatteryDischargeModel]
    recharge_model_factory: Callable[[], BatteryRechargeModel]

    rotor_speed: float = 1788.53


# ---------------------------------------------------------------------------
# Battery factories
# ---------------------------------------------------------------------------

def constant_battery(
    capacity_wh: float,
    power_w: float = 180.0,
    recharge_power_w: float = 500.0,
) -> Battery:
    return Battery(
        capacity_wh=capacity_wh,
        discharge_model=ConstantPowerDischarge(
            power_w=power_w,
        ),
        recharge_model=ConstantRecharge(
            power_w=recharge_power_w,
        ),
    )


def payload_wind_battery(
    capacity_wh: float,
    base_power_w: float = 180.0,
    payload_power_per_kg: float = 20.0,
    wind_power_per_mps: float = 15.0,
    recharge_power_w: float = 500.0,
) -> Battery:
    return Battery(
        capacity_wh=capacity_wh,
        discharge_model=PayloadWindDischarge(
            base_power_w=base_power_w,
            payload_power_per_kg=payload_power_per_kg,
            wind_power_per_mps=wind_power_per_mps,
        ),
        recharge_model=ConstantRecharge(
            power_w=recharge_power_w,
        ),
    )

DRONE_CONFIGS = [
    DroneConfig(
        name="Light",
        cruise_speed=12.0,
        payload_capacity=2.0,
        rotor_speed=1650.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=100.0,
        discharge_model_factory=lambda: ConstantPowerDischarge(
            power_w=150.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(
            power_w=400.0,
        ),
    ),

    DroneConfig(
        name="Standard",
        cruise_speed=15.0,
        payload_capacity=5.0,
        rotor_speed=1788.53,
        rotorpy_params=quad_params,
        battery_capacity_wh=100.0,
        discharge_model_factory=lambda: ConstantPowerDischarge(
            power_w=180.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(
            power_w=500.0,
        ),
    ),

    DroneConfig(
        name="Heavy",
        cruise_speed=15.0,
        payload_capacity=8.0,
        rotor_speed=1850.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=150.0,
        discharge_model_factory=lambda: PayloadWindDischarge(
            base_power_w=220.0,
            payload_power_per_kg=30.0,
            wind_power_per_mps=20.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(
            power_w=600.0,
        ),
    ),

    DroneConfig(
        name="Fast",
        cruise_speed=22.0,
        payload_capacity=3.0,
        rotor_speed=1950.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=140.0,
        discharge_model_factory=lambda: PayloadWindDischarge(
            base_power_w=250.0,
            payload_power_per_kg=25.0,
            wind_power_per_mps=18.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(
            power_w=700.0,
        ),
    ),

    DroneConfig(
        name="Endurance",
        cruise_speed=12.0,
        payload_capacity=4.0,
        rotor_speed=1600.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=200.0,
        discharge_model_factory=lambda: PayloadWindDischarge(
            base_power_w=160.0,
            payload_power_per_kg=18.0,
            wind_power_per_mps=12.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(
            power_w=450.0,
        ),
    ),
]

# ---------------------------------------------------------------------------
# Vehicle construction
# ---------------------------------------------------------------------------

def build_vehicle(
    home_position: Vector3,
    wind: Wind,
    config: DroneConfig,
) -> RotorPyVehicleAdapter:

    initial_state = {
        "x": np.array([
            home_position.x,
            home_position.y,
            home_position.z,
        ]),
        "v": np.zeros(3),
        "q": np.array([0.0, 0.0, 0.0, 1.0]),
        "w": np.zeros(3),
        "wind": np.zeros(3),
        "rotor_speeds": np.array([
            config.rotor_speed,
            config.rotor_speed,
            config.rotor_speed,
            config.rotor_speed,
        ]),
    }

    rotorpy_vehicle = Multirotor(
        config.rotorpy_params,
        initial_state=initial_state,
    )

    rotorpy_controller = SE3Control(
        config.rotorpy_params,
    )

    rotorpy_environment = Environment(
        vehicle=rotorpy_vehicle,
        controller=rotorpy_controller,
        trajectory=HoverTraj(
            x0=np.array([
                home_position.x,
                home_position.y,
                home_position.z,
            ])
        ),
        wind_profile=ConstantWind(
            wx=wind.velocity.x,
            wy=wind.velocity.y,
            wz=wind.velocity.z,
        ),
        sim_rate=100,
    )

    battery = Battery(
        capacity_wh=config.battery_capacity_wh,
        discharge_model=config.discharge_model_factory(),
        recharge_model=config.recharge_model_factory(),
    )

    return RotorPyVehicleAdapter(
        rotorpy_vehicle=rotorpy_vehicle,
        rotorpy_environment=rotorpy_environment,
        rotorpy_controller=rotorpy_controller,
        config=config,
        home_position=home_position,
        battery=battery,
        wind=wind,
    )

# ---------------------------------------------------------------------------
# Fleet construction
# ---------------------------------------------------------------------------

def build_fleet(
    base: Vector3,
    wind: Wind,
    configs: list[DroneConfig],
    spacing: float = 1.5,
) -> list[RotorPyVehicleAdapter]:

    vehicles = []

    for i, config in enumerate(configs):
        home_position = Vector3(
            base.x + i * spacing,
            base.y,
            base.z,
        )

        vehicles.append(
            build_vehicle(
                home_position=home_position,
                wind=wind,
                config=config,
            )
        )

    return vehicles

def build_diverse_fleet(
    base: Vector3,
    wind: Wind,
    fleet_size: int,
    spacing: float = 1.5,
) -> list[RotorPyVehicleAdapter]:

    configs = [
        DRONE_CONFIGS[i % len(DRONE_CONFIGS)]
        for i in range(fleet_size)
    ]

    return build_fleet(
        base=base,
        wind=wind,
        configs=configs,
        spacing=spacing,
    )
# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------

FLEET_SIZE = 5


def build_application():

    map = MapData("maps/richmond_small_3.json")
    map_bounds = map.get_bounds()

    x0, y0, x1, y1 = map_bounds

    base = Vector3(
        (x0 + x1) / 2,
        (y0 + y1) / 2,
        2.0,
    )

    scenario = Scenario(
        bounds=map_bounds,
        base=base,
        wind=Wind(Vector3(2.0, 0.0, 0.0)),
        obstacles=map.get_obstacles(),
        orders=[],
    )

    environment = ScenarioManager(scenario)

    vehicles = build_diverse_fleet(
        base=base,
        wind=scenario.wind,
        fleet_size=FLEET_SIZE,
    )

    planner = MissionPlanner(environment)

    mission_manager = MissionManager(
        vehicles=vehicles,
        environment=environment,
        planner=planner,
    )

    simulation = SimulationManager(
        vehicles=vehicles,
        environment=environment,
    )

    ui = PygameUI(
        scenario_manager=environment,
        mission_manager=mission_manager,
        simulation=simulation,
        map_data=map,
    )

    return ui


if __name__ == "__main__":
    app = build_application()
    app.run()