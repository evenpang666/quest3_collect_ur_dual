"""Start the Quest bridge, mapper, RTDE backend and recorder for either mode."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware = LaunchConfiguration("hardware_config")
    recorder = LaunchConfiguration("recorder_config")
    output = LaunchConfiguration("output_dir")
    front_topic = LaunchConfiguration("front_topic")
    return LaunchDescription([
        DeclareLaunchArgument("hardware_config"),
        DeclareLaunchArgument("recorder_config"),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("front_topic", default_value="/front_camera/camera/color/image_raw"),
        Node(package="quest3_robot_data", executable="rtde_servo", output="screen", arguments=["--config", hardware]),
        Node(package="quest3_robot_data", executable="pose_mapper", output="screen", arguments=["--config", hardware]),
        Node(package="quest3_robot_data", executable="webxr_bridge", output="screen", arguments=["--front-topic", front_topic]),
        Node(package="quest3_robot_data", executable="record_dataset", output="screen",
             arguments=["--config", recorder, "--hardware-config", hardware, "--output-dir", output]),
    ])
