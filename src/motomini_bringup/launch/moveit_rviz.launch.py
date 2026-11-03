import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

for var in ("GTK_PATH", "GIO_MODULE_DIR", "LOCPATH"):
    os.environ.pop(var, None)
if "LD_LIBRARY_PATH" in os.environ:
    clean_paths = [
        p for p in os.environ["LD_LIBRARY_PATH"].split(":")
        if "/snap/" not in p
    ]
    os.environ["LD_LIBRARY_PATH"] = ":".join(clean_paths)


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "motomini_workcell", package_name="motomini_bringup"
    ).to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
