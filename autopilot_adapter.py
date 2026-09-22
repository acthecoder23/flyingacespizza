# autopilot_adapter.py

from contracts import *


class AutopilotVehicleAdapter(VehicleInterface):

    def __init__(self, mavsdk_system):
        self.system = mavsdk_system

        self.state = VehicleState(
            position=Vector3(0, 0, 0),
            battery_percent=100.0,
            state=DroneState.LANDED,
        )

    def get_state(self) -> VehicleState:
        return self.state

    def upload_route(self, route: Route) -> None:
        """
        Translate Route into MAVSDK mission items.

        This is the only place that should know about MAVSDK.
        """
        pass

    def start_mission(self) -> None:
        # MAVSDK mission.start_mission()
        pass

    def pause_mission(self) -> None:
        # MAVSDK hold()
        pass

    def abort_mission(self) -> None:
        # MAVSDK action.land() / return_to_launch()
        pass

    def return_to_base(self) -> None:
        # MAVSDK action.return_to_launch()
        pass

    def advance(self, dt: float) -> None:
        """
        Autopilot backend is externally advancing.
        Telemetry would be polled asynchronously.
        """
        pass

    def reset(self) -> None:
        # MAVSDK: land/disarm, then reset local shadow state.
        self.state = VehicleState(
            position=Vector3(0, 0, 0),
            battery_percent=100.0,
            state=DroneState.LANDED,
        )