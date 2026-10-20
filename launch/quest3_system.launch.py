"""Start the Quest bridge, mapper, RTDE backend and recorder for either mode."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware = LaunchConfiguration("hardware_config")
    recorder = LaunchConfiguration("recorder_config")
    output = LaunchConfiguration("output_dir")
    task_description = LaunchConfiguration("task_description")
    start_recorder = LaunchConfiguration("start_recorder")
    record_depth = LaunchConfiguration("record_depth")
    front_topic = LaunchConfiguration("front_topic")
    left_wrist_topic = LaunchConfiguration("left_wrist_topic")
    right_wrist_topic = LaunchConfiguration("right_wrist_topic")
    vr_stream_fps = LaunchConfiguration("vr_stream_fps")
    vr_wrist_stream_fps = LaunchConfiguration("vr_wrist_stream_fps")
    vr_jpeg_quality = LaunchConfiguration("vr_jpeg_quality")
    return LaunchDescription([
        DeclareLaunchArgument("hardware_config"),
        DeclareLaunchArgument("recorder_config"),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("task_description"),
        DeclareLaunchArgument("start_recorder", default_value="true"),
        DeclareLaunchArgument("record_depth", default_value="false", choices=["true", "false"],
                             description="Include depth in recordings; match camera enable_depth"),
        DeclareLaunchArgument("front_topic", default_value="/front_camera/camera/color/image_raw"),
        DeclareLaunchArgument("left_wrist_topic", default_value="/left_wrist_camera/camera/color/image_raw"),
        DeclareLaunchArgument("right_wrist_topic", default_value="/right_wrist_camera/camera/color/image_raw"),
        # A stable 15 FPS preview is preferable to a nominal 30 FPS stream
        # whose Python JPEG/HTTP path drops frames under teleoperation load.
        DeclareLaunchArgument("vr_stream_fps", default_value="15"),
        # Wrist previews are deliberately lower rate to keep the front view responsive.
        DeclareLaunchArgument("vr_wrist_stream_fps", default_value="10"),
        DeclareLaunchArgument("vr_jpeg_quality", default_value="90"),
        Node(package="quest3_robot_data", executable="rtde_servo", output="screen", arguments=["--config", hardware]),
        Node(package="quest3_robot_data", executable="pose_mapper", output="screen", arguments=["--config", hardware]),
        Node(package="quest3_robot_data", executable="webxr_bridge", output="screen", arguments=[
            "--front-topic", front_topic,
            "--recording-enabled", start_recorder,
            "--left-wrist-topic", left_wrist_topic,
            "--right-wrist-topic", right_wrist_topic,
            "--stream-fps", vr_stream_fps,
            "--wrist-stream-fps", vr_wrist_stream_fps,
            "--jpeg-quality", vr_jpeg_quality,
        ]),
        Node(package="quest3_robot_data", executable="record_dataset", output="screen",
             condition=IfCondition(start_recorder),
             arguments=[
                 "--config", recorder,
                 "--hardware-config", hardware,
                 "--output-dir", output,
                 "--task-description", task_description,
                 "--record-depth", record_depth,
             ]),
    ])
