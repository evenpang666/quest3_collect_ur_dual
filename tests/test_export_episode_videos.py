from scripts.export_episode_videos import output_fps


def test_export_defaults_to_30_fps_per_synchronized_sample():
    assert output_fps(None) == 30.0
    assert output_fps(24.0) == 24.0
