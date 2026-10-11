from pathlib import Path

from setuptools import find_packages, setup

package_name = "quest3_robot_data"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/quest3_robot_data"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/dual_arm.yaml", "config/single_arm.yaml"]),
        (f"share/{package_name}/config", [
            "config/hardware.yaml",
            "config/hardware_single.yaml",
            "config/hardware_left_debug.yaml",
            "config/hardware_left_teleop.yaml",
            "config/hardware_dual_teleop.yaml",
            "config/single_arm_debug.yaml",
            "config/dual_arm_front_only.yaml",
        ]),
        (f"share/{package_name}/config/calibration", [
            str(path) for pattern in ("*.md", "*example.yaml")
            for path in Path("config/calibration").glob(pattern) if path.is_file()
        ]),
        (f"share/{package_name}/launch", ["launch/quest3_system.launch.py", "launch/realsense_four.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "h5py", "aiohttp"],
    zip_safe=True,
    package_data={package_name: ["web/index.html"]},
    entry_points={"console_scripts": [
        "record_dataset = quest3_robot_data.recorder:main",
        "webxr_bridge = quest3_robot_data.webxr_bridge:main",
        "view_front_camera = quest3_robot_data.view_front_camera:main",
        "view_realsense_cameras = quest3_robot_data.view_front_camera:main",
        "pose_mapper = quest3_robot_data.pose_mapper:main",
        "rtde_servo = quest3_robot_data.rtde_servo:main",
        "joint_motion_probe = quest3_robot_data.joint_motion_probe:main",
        "calibrate_tcp = quest3_robot_data.calibrate_tcp:main",
        "calibrate_camera = quest3_robot_data.calibrate_camera:main",
        "calibrate_camera_charuco = quest3_robot_data.calibrate_camera_charuco:main",
        "calibrate_dual_camera_charuco = quest3_robot_data.calibrate_dual_camera_charuco:main",
        "preflight = quest3_robot_data.preflight:main",
        "inspect_episode = scripts.inspect_episode:main",
        "export_episode_videos = scripts.export_episode_videos:main",
        "export_task_episode_videos = scripts.export_task_episode_videos:main",
        "export_folder_episode_videos = scripts.export_folder_episode_videos:main",
    ]},
)
