"""The dedicated avoidance worker must not outlive its teleop launcher."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.skipif(sys.platform != "linux", reason="uses Linux parent-death signal")
def test_worker_exits_when_launcher_is_killed():
    launcher_code = """
import os
import subprocess
import sys
env = os.environ.copy()
env['QUEST3_AVOIDANCE_PARENT_PID'] = str(os.getpid())
worker = subprocess.Popen([
    sys.executable, '-c',
    'from quest3_robot_data.teleop_avoidance_server import _exit_with_launcher; '
    '_exit_with_launcher(); import time; time.sleep(30)',
], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print(worker.pid, flush=True)
worker.wait()
"""
    launcher = subprocess.Popen([sys.executable, "-c", launcher_code], stdout=subprocess.PIPE,
                                text=True)
    worker_pid = None
    try:
        worker_pid = int(launcher.stdout.readline().strip())
        assert launcher.poll() is None
        launcher.kill()
        launcher.wait(timeout=5)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            status = Path(f"/proc/{worker_pid}/stat")
            if not status.exists() or status.read_text().split()[2] == "Z":
                break
            time.sleep(0.05)
        else:
            pytest.fail("avoidance worker survived its launcher")
    finally:
        if launcher.poll() is None:
            launcher.kill()
            launcher.wait(timeout=5)
        if worker_pid is not None:
            try:
                os.kill(worker_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
