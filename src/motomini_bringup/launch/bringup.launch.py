#!/usr/bin/env python3
"""
File Bringup Duy Nhat (bringup.launch.py) cho he thong Yaskawa MotoMINI:
Tu dong khoi chay cung luc:
  1. Mo hinh 3D URDF (motomini_description) + MoveIt 2 & RViz2 (motomini_moveit_config)
  2. Node MP3300 Machine Controller, 2 Bang tai Encoder & Mam xoay (motomini_control: virtual_conveyor.py)
  3. Node Giao dien YRC1000micro Teach Pendant #1 cho Robot 1 - R1 (motomini_control: inform_teach_pendant.py R1)
  4. Node Giao dien YRC1000micro Teach Pendant #2 cho Robot 2 - R2 (motomini_control: inform_teach_pendant.py R2)
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
        "motomini_workcell", package_name="motomini_moveit_config"
    ).to_moveit_configs()

    ld = generate_demo_launch(moveit_config)

    # 1. Node MP3300 Machine Controller + 2 Bang tai Encoder & Mam xoay
    conveyor_node = Node(
        package="motomini_control",
        executable="virtual_conveyor.py",
        name="virtual_conveyor",
        output="screen",
    )

    # 2. Giao dien YRC1000micro Teach Pendant #1 (Điều khiển Cánh tay Robot 1 - R1)
    teach_pendant_r1_node = Node(
        package="motomini_control",
        executable="inform_teach_pendant.py",
        name="inform_teach_pendant_r1",
        arguments=["R1"],
        output="screen",
    )

    # 3. Giao dien YRC1000micro Teach Pendant #2 (Điều khiển Cánh tay Robot 2 - R2)
    teach_pendant_r2_node = Node(
        package="motomini_control",
        executable="inform_teach_pendant.py",
        name="inform_teach_pendant_r2",
        arguments=["R2"],
        output="screen",
    )

    ld.add_action(conveyor_node)
    ld.add_action(teach_pendant_r1_node)
    ld.add_action(teach_pendant_r2_node)

    return ld
