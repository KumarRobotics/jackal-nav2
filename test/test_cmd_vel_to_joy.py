from jackal_nav2.cmd_vel_to_joy import CmdVelToJoy

import pytest
from sensor_msgs.msg import Joy


def test_velocity_to_axis_preserves_physical_scale():
    assert CmdVelToJoy.velocity_to_axis(0.5, 2.0) == pytest.approx(0.25)
    assert CmdVelToJoy.velocity_to_axis(-0.3, 0.75) == pytest.approx(-0.4)


def test_velocity_to_axis_clamps():
    assert CmdVelToJoy.velocity_to_axis(3.0, 2.0) == 1.0
    assert CmdVelToJoy.velocity_to_axis(-3.0, 2.0) == -1.0


def test_velocity_to_axis_rejects_invalid_calibration():
    with pytest.raises(ValueError):
        CmdVelToJoy.velocity_to_axis(0.1, 0.0)


def test_bridge_output_marker_distinguishes_generated_joy():
    generated = Joy()
    generated.header.frame_id = CmdVelToJoy.DEFAULT_OUTPUT_FRAME_ID
    physical = Joy()

    assert CmdVelToJoy.is_bridge_output(
        generated, CmdVelToJoy.DEFAULT_OUTPUT_FRAME_ID
    )
    assert not CmdVelToJoy.is_bridge_output(
        physical, CmdVelToJoy.DEFAULT_OUTPUT_FRAME_ID
    )


def test_manual_override_only_matches_physical_manual_axis_value():
    manual = Joy()
    manual.axes = [0.0, 0.0, 0.0, 0.0, -1.0]
    automatic = Joy()
    automatic.axes = [0.0, 0.0, 0.0, 0.0, 1.0]
    undersized = Joy()
    undersized.axes = [0.0] * 4

    assert CmdVelToJoy.is_manual_override(manual, 4, -1.0)
    assert not CmdVelToJoy.is_manual_override(automatic, 4, -1.0)
    assert not CmdVelToJoy.is_manual_override(undersized, 4, -1.0)


def test_only_positive_physical_mode_axis_allows_autonomy():
    manual = Joy()
    manual.axes = [0.0, 0.0, 0.0, 0.0, -1.0]
    neutral = Joy()
    neutral.axes = [0.0] * 5
    automatic = Joy()
    automatic.axes = [0.0, 0.0, 0.0, 0.0, 1.0]
    undersized = Joy()
    undersized.axes = [0.0] * 4

    assert not CmdVelToJoy.operator_allows_autonomy(manual, 4)
    assert not CmdVelToJoy.operator_allows_autonomy(neutral, 4)
    assert CmdVelToJoy.operator_allows_autonomy(automatic, 4)
    assert not CmdVelToJoy.operator_allows_autonomy(undersized, 4)


def test_physical_auto_enables_generated_output_and_manual_blocks_it():
    class BridgeState:
        output_frame_id = CmdVelToJoy.DEFAULT_OUTPUT_FRAME_ID
        manual_override_axis = 4
        manual_override_value = -1.0
        block_autonomy = True
        is_bridge_output = staticmethod(CmdVelToJoy.is_bridge_output)
        is_manual_override = staticmethod(CmdVelToJoy.is_manual_override)
        operator_allows_autonomy = staticmethod(
            CmdVelToJoy.operator_allows_autonomy)

    bridge = BridgeState()
    physical_auto = Joy()
    physical_auto.header.frame_id = "physical_rc"
    physical_auto.axes = [0.0, 0.0, 0.0, 0.0, 1.0]
    generated = Joy()
    generated.header.frame_id = CmdVelToJoy.DEFAULT_OUTPUT_FRAME_ID
    generated.axes = [0.0] * 8
    physical_manual = Joy()
    physical_manual.header.frame_id = "physical_rc"
    physical_manual.axes = [0.0, 0.0, 0.0, 0.0, -1.0]

    CmdVelToJoy.joy_cb(bridge, physical_auto)
    assert bridge.block_autonomy is False
    CmdVelToJoy.joy_cb(bridge, generated)
    assert bridge.block_autonomy is False
    CmdVelToJoy.joy_cb(bridge, physical_manual)
    assert bridge.block_autonomy is True


def test_command_axes_use_teleop_forwarding_mode():
    axes = CmdVelToJoy.command_axes(
        linear_velocity=0.5,
        angular_velocity=-0.3,
        linear_speed_at_full_axis=2.0,
        angular_speed_at_full_axis=0.75,
        mode_axis=4,
    )

    assert axes[3] == pytest.approx(0.25)
    assert axes[2] == pytest.approx(-0.4)
    assert axes[4] == 0.0
