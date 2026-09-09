"""Start configured D435i wrist and D455 scene cameras by serial number."""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera(name, serial, depth):
    # The fixed front D455 supplies Quest with sharp 1280x720 RGB, while its
    # depth remains 640x480 to reduce USB load and avoid the D455 high-res
    # depth-stream startup failures seen during teleoperation.
    is_front = name == "front"
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=f"{name}_camera",
        name="camera",
        output="screen",
        parameters=[{
            # realsense-ros requires the underscore prefix to prevent numeric
            # serials from being auto-converted by ROS parameter parsing.
            "serial_no": "_" + str(serial).lstrip("_"),
            "enable_color": True,
            "enable_depth": depth,
            "enable_infra1": False,
            "enable_infra2": False,
            # D435i/D455 motion sensors are not part of the recorded schema.
            # Explicitly disable them so IIO/HID permissions cannot prevent
            # RGB-D bring-up on a fresh workstation.
            "enable_gyro": False,
            "enable_accel": False,
            "unite_imu_method": 0,
            "align_depth.enable": depth,
            "rgb_camera.color_profile": "1280x720x30" if is_front else "640x480x30",
            "depth_module.depth_profile": "640x480x30",
        }],
    )


def launch_cameras(context):
    path = Path(LaunchConfiguration("hardware_config").perform(context)).expanduser().resolve()
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    nodes = []
    for name, values in config["cameras"].items():
        serial = str(values["serial"])
        if "CHANGE_ME" in serial:
            raise RuntimeError(f"configure the serial number for camera {name}")
        nodes.append(camera(name, serial, name in {"front", "side"}))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_config"),
        OpaqueFunction(function=launch_cameras),
    ])
