import json
import math
from pathlib import Path

from jackal_nav2.motion_metrics import (
    MotionReport,
    TrackingSample,
    VelocitySeries,
    jerk_value_for_plot,
    robust_jerk_plot_limit,
    should_plot_series,
    value_for_plot,
)

import pytest


def test_velocity_series_derives_acceleration_and_summary():
    series = VelocitySeries("test", acceleration_filter_alpha=1.0)
    series.add(0.0, 0.0, 0.0)
    sample = series.add(1.0, 1.0, -0.5)
    final_sample = series.add(2.0, 1.0, -0.5)

    assert sample.linear_acceleration == pytest.approx(1.0)
    assert sample.angular_acceleration == pytest.approx(-0.5)
    assert final_sample.linear_jerk == pytest.approx(-1.0)
    assert final_sample.angular_jerk == pytest.approx(0.5)
    summary = series.summary()
    assert summary["sample_count"] == 3
    assert summary["average_sample_rate_hz"] == pytest.approx(1.0)
    assert summary["estimated_distance_travelled_m"] == pytest.approx(1.5)
    assert summary["linear_velocity_mps"]["peak_absolute"] == pytest.approx(1.0)
    assert summary["linear_jerk_mps3"]["mean"] == pytest.approx(-1.0)


@pytest.mark.parametrize(
    ("field", "value", "is_filtered"),
    [
        ("linear_acceleration", -1.01, True),
        ("linear_acceleration", 2.01, True),
        ("angular_acceleration", -800.0, True),
        ("angular_acceleration", 800.0, True),
        ("linear_acceleration", -1.0, False),
        ("angular_acceleration", 2.0, False),
        ("linear_velocity", 800.0, False),
    ],
)
def test_value_for_plot_filters_only_out_of_range_acceleration(
    field, value, is_filtered
):
    plotted_value = value_for_plot(field, value)
    assert math.isnan(plotted_value) is is_filtered


def test_robust_jerk_plot_limit_ignores_rare_spikes():
    values = [1.0] * 98 + [1000.0, -2000.0]

    limit = robust_jerk_plot_limit(values)

    assert limit == pytest.approx(1.0)
    assert jerk_value_for_plot(1.0, limit) == pytest.approx(1.0)
    assert math.isnan(jerk_value_for_plot(1000.0, limit))
    assert robust_jerk_plot_limit([]) is None


@pytest.mark.parametrize(
    ("source", "field", "expected"),
    [
        ("odometry", "linear_acceleration", False),
        ("odometry", "angular_acceleration", False),
        ("odometry", "linear_velocity", False),
        ("odometry", "angular_velocity", False),
        ("odometry", "linear_jerk", False),
        ("odometry", "angular_jerk", False),
        ("nav_command", "linear_acceleration", True),
        ("smoothed_command", "angular_acceleration", True),
        ("nav_command", "linear_jerk", True),
        ("smoothed_command", "angular_jerk", True),
    ],
)
def test_should_plot_series_excludes_odometry_from_motion_profiles(
    source, field, expected
):
    assert should_plot_series(source, field) is expected


def test_motion_report_writes_plot_data_and_summary(tmp_path: Path):
    series = {
        name: VelocitySeries(
            name,
            acceleration_filter_alpha=1.0,
            calculate_jerk=name != "odometry",
        )
        for name in ("nav_command", "smoothed_command", "odometry")
    }
    for velocity_series in series.values():
        velocity_series.add(0.0, 0.0, 0.0)
        velocity_series.add(1.0, 0.5, 0.1)
        velocity_series.add(2.0, 0.5, 0.1)
    tracking = [TrackingSample(1.0, 0.5, 0.4, 0.1, 0.08)]
    report = MotionReport(tmp_path, series, tracking, {"test_run": True})

    artifacts = report.write("test")

    assert all(path.is_file() for path in artifacts)
    assert {path.name for path in artifacts} == {
        "jerk_profile.png",
        "summary.json",
        "tracking_error_samples.csv",
        "velocity_and_acceleration.png",
        "velocity_samples.csv",
        "velocity_tracking_error.png",
    }
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["report_reason"] == "test"
    assert summary["smoothed_command_tracking"]["sample_count"] == 1
    assert "linear_jerk_mps3" in summary["streams"]["nav_command"]
    assert "angular_jerk_radps3" in summary["streams"]["smoothed_command"]
    assert "linear_jerk_mps3" not in summary["streams"]["odometry"]
    csv_header = (tmp_path / "velocity_samples.csv").read_text().splitlines()[0]
    assert "linear_jerk" in csv_header
    assert "angular_jerk" in csv_header
