import numpy as np

from quest3_robot_data.teleop_core import ControllerGate, ControllerState


def tracked_hand(squeeze=0.0):
    return ControllerState(
        position=np.zeros(3), orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        squeeze=squeeze, trigger=0.0,
    )


def test_dual_controller_gate_requires_both_fresh_tracked_hands():
    gate = ControllerGate(("left", "right"), 0.25)
    assert not gate.ready(0)
    assert gate.missing(0) == ("left", "right")

    gate.observe({"left": tracked_hand(), "right": None}, 1_000_000_000)
    assert not gate.ready(1_000_000_000)
    assert gate.missing(1_000_000_000) == ("right",)

    gate.observe({"left": tracked_hand(), "right": tracked_hand()}, 1_100_000_000)
    assert gate.ready(1_100_000_000)
    assert gate.ready(1_350_000_000)
    assert not gate.ready(1_350_000_001)
    assert gate.missing(1_350_000_001) == ("left", "right")


def test_single_arm_gate_only_requires_left_hand():
    gate = ControllerGate(("left",), 0.25)
    gate.observe({"left": tracked_hand()}, 100)
    assert gate.ready(100)


def test_nonfinite_controller_input_is_not_tracked():
    assert ControllerState.from_dict({
        "position": [0, 0, 0], "orientation": [0, 0, 0, 1],
        "squeeze": float("nan"), "trigger": 0,
    }) is None
