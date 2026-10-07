from __future__ import annotations

import argparse

from pygame_ui import PygameUI
from simulation import Simulation


def run_interactive(scenario_path: str) -> None:
    simulation = Simulation.from_json(scenario_path)
    ui = PygameUI(
        scenario_manager=simulation.environment,
        mission_manager=simulation.mission_manager,
        simulation=simulation,
    )
    ui.run()


def run_headless(scenario_path: str) -> None:
    simulation = Simulation.from_json(scenario_path)

    print(f"Loaded scenario: {scenario_path}")
    print(f"Simulation time: {simulation.time_seconds:.1f}s")
    print(f"Fleet size: {len(simulation.vehicles)}")
    print(f"Orders: {len(simulation.scenario.orders)}")
    print()
    print("Headless execution is not implemented yet.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pizza Drone Simulation"
    )

    parser.add_argument(
        "scenario",
        help="Path to the simulation scenario JSON file",
    )

    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without the Pygame visualization",
    )

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.headless:
        run_headless(args.scenario)
    else:
        run_interactive(args.scenario)


if __name__ == "__main__":
    main()