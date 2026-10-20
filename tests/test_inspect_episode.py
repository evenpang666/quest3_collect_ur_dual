import numpy as np
import pytest

from scripts.inspect_episode import timestamp_diagnostics


def test_timestamp_diagnostics_exposes_gaps_in_nominal_30fps_recording():
    stamps = np.array([0, 1, 2, 5, 6]) * (1e9 / 30)
    stats = timestamp_diagnostics(stamps)
    assert stats["fps"] == pytest.approx(20)
    assert stats["long_gaps"] == 1
    assert stats["estimated_missing"] == 2
    assert stats["non_monotonic"] == 0


def test_short_recordings_and_reversed_unsigned_stamps_do_not_wrap():
    assert timestamp_diagnostics([])["fps"] == 0
    assert timestamp_diagnostics([123])["estimated_missing"] == 0
    stats = timestamp_diagnostics(np.array([100_000_000, 66_666_667, 66_666_667], np.uint64))
    assert stats["non_monotonic"] == 2
    assert stats["estimated_missing"] == 0


@pytest.mark.parametrize("fps", [0, -30, np.nan, np.inf])
def test_invalid_reference_rates_are_rejected(fps):
    with pytest.raises(ValueError, match="FPS"):
        timestamp_diagnostics([1, 2], fps)
