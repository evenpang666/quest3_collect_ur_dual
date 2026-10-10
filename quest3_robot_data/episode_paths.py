"""Directory naming for recorded episodes."""
from __future__ import annotations

import re
from pathlib import Path


def task_directory_name(description: str) -> str:
    """Turn a task label into one safe, lowercase directory component."""
    name = re.sub(r"[^\w-]+", "_", description.lower()).strip("_")
    if not name:
        raise ValueError("--task-description must contain a letter or number")
    return name


def episode_output_dir(base_dir: Path, mode: str, task_description: str) -> Path:
    base_dir = base_dir.expanduser().resolve()
    return base_dir / task_directory_name(task_description) if mode == "dual" else base_dir
