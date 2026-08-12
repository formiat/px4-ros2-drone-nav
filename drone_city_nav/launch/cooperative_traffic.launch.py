import runpy
from pathlib import Path


_MULTI_VEHICLE_LAUNCH = runpy.run_path(
    str(Path(__file__).with_name("multi_vehicle.launch.py"))
)


def generate_launch_description():
    return _MULTI_VEHICLE_LAUNCH["generate_multi_vehicle_launch_description"](
        "cooperative_traffic"
    )
