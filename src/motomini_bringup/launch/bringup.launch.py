#!/usr/bin/env python3
"""
File Bringup Duy Nhat (bringup.launch.py) cho he thong Yaskawa MotoMINI:
Tu dong khoi chay cung luc:
  1. Robot State Publisher + MoveIt 2 (move_group) + ros2_control + RViz2
  2. Node Bang tai ao & Vat mau da hinh dang (virtual_conveyor.py)
  3. Cua so Giao dien Bang Teach Pendant INFORM III (inform_teach_pendant.py)
"""

import os
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

# Lam sach bien moi truong Snap (khi chay tu VS Code Terminal)
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

    ld = generate_demo_launch(moveit_config)

    # Tu dong khoi chay Node Bang tai ao & Vat mau
    conveyor_node = Node(
        package="motomini_bringup",
        executable="virtual_conveyor.py",
        name="virtual_conveyor",
        output="screen",
    )

    # Tu dong khoi chay Giao dien Bang Teach Pendant GUI
    teach_pendant_node = Node(
        package="motomini_bringup",
        executable="inform_teach_pendant.py",
        name="inform_teach_pendant",
        output="screen",
    )

    ld.add_action(conveyor_node)
    ld.add_action(teach_pendant_node)

    return ld

