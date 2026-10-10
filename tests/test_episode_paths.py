import pytest

from quest3_robot_data.episode_paths import episode_output_dir, task_directory_name


def test_dual_episode_path_uses_normalized_task_name(tmp_path):
    assert task_directory_name("Pick Up Cube") == "pick_up_cube"
    assert task_directory_name("双臂 协作") == "双臂_协作"
    assert task_directory_name("Pick/Place") == "pick_place"
    assert episode_output_dir(tmp_path, "dual", "Pick Up Cube") == tmp_path / "pick_up_cube"
    assert episode_output_dir(tmp_path, "single", "Pick Up Cube") == tmp_path


def test_task_name_cannot_be_empty_or_escape_its_parent():
    with pytest.raises(ValueError):
        task_directory_name(" / ")
    assert task_directory_name("../Other Task") == "other_task"
