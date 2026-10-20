"""Run copied launch scripts against fake commands, without touching hardware."""
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(("script_name", "confirmation"), [
    ("start_dual_vr_teleop.sh", "MOVE_BOTH_ARMS"),
    ("start_left_vr_teleop.sh", "MOVE_LEFT_ARM"),
])
@pytest.mark.parametrize("record_depth", [False, True])
def test_script_passes_matching_depth_flags_to_camera_and_recorder(tmp_path, script_name, confirmation, record_depth):
    project = tmp_path / "project"
    scripts = project / "real_scripts"
    scripts.mkdir(parents=True)
    (scripts / "ros_environment.sh").write_text("true\n")
    (scripts / "quest_keep_awake.sh").write_text("exit 0\n")
    text = (ROOT / "real_scripts" / script_name).read_text()
    project_assignment = f'PROJECT_DIR="{ROOT}"'
    assert project_assignment in text
    text = text.replace(project_assignment, f'PROJECT_DIR="{project}"')
    text = text.replace("/tmp/quest3_dual_vr_teleop.lock", str(tmp_path / "teleop.lock"))
    launcher = scripts / script_name
    launcher.write_text(text)
    commands = tmp_path / "bin"
    commands.mkdir()
    for command, body in {
        "ros2": 'printf "%s\\n" "$*" >> "$TELEOP_DEPTH_TEST_LOG"\n',
        "adb": "exit 0\n", "sleep": "exit 0\n",
    }.items():
        executable = commands / command
        executable.write_text("#!/usr/bin/env bash\n" + body)
        executable.chmod(0o755)
    log = tmp_path / "commands.log"
    env = dict(os.environ, PATH=f"{commands}:{os.environ['PATH']}", TELEOP_DEPTH_TEST_LOG=str(log))
    env.pop("CONDA_PREFIX", None)
    args = ["bash", str(launcher), "--confirm", confirmation, "--task", "depth test"]
    if record_depth:
        args.append("--record-depth")

    result = subprocess.run(args, env=env, text=True, capture_output=True, timeout=10)

    assert result.returncode == 0, result.stdout + result.stderr
    calls = log.read_text().splitlines()
    camera = next(line for line in calls if "realsense_four.launch.py" in line)
    system = next(line for line in calls if "quest3_system.launch.py" in line)
    expected = str(record_depth).lower()
    assert f"enable_depth:={expected}" in camera
    assert f"record_depth:={expected}" in system
