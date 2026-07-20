from jackal_nav2.cmd_vel_to_joy import CmdVelToJoy

import pytest


def test_velocity_to_axis_preserves_physical_scale():
    assert CmdVelToJoy.velocity_to_axis(0.5, 2.0) == pytest.approx(0.25)
    assert CmdVelToJoy.velocity_to_axis(-0.3, 0.75) == pytest.approx(-0.4)


def test_velocity_to_axis_clamps():
    assert CmdVelToJoy.velocity_to_axis(3.0, 2.0) == 1.0
    assert CmdVelToJoy.velocity_to_axis(-3.0, 2.0) == -1.0


def test_velocity_to_axis_rejects_invalid_calibration():
    with pytest.raises(ValueError):
        CmdVelToJoy.velocity_to_axis(0.1, 0.0)
