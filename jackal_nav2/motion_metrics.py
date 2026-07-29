"""Data collection and report generation for Jackal motion analysis."""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
from statistics import fmean, pstdev
from typing import Iterable

ACCELERATION_PLOT_MIN = -1.0
ACCELERATION_PLOT_MAX = 2.0
JERK_PLOT_PERCENTILE = 92.0
JERK_PLOT_MIN_ABS_LIMIT = 1.0e-6
ACCELERATION_FIELDS = {"linear_acceleration", "angular_acceleration"}
JERK_FIELDS = {"linear_jerk", "angular_jerk"}


@dataclass
class VelocitySample:
    """One velocity observation and its derived acceleration."""

    time: float
    linear_velocity: float
    angular_velocity: float
    linear_acceleration: float | None
    angular_acceleration: float | None
    raw_linear_acceleration: float | None
    raw_angular_acceleration: float | None
    linear_jerk: float | None
    angular_jerk: float | None


@dataclass
class TrackingSample:
    """A target/actual velocity pair captured at the actual sample time."""

    time: float
    target_linear: float
    actual_linear: float
    target_angular: float
    actual_angular: float

    @property
    def linear_error(self) -> float:
        return self.actual_linear - self.target_linear

    @property
    def angular_error(self) -> float:
        return self.actual_angular - self.target_angular


def _signal_statistics(values: Iterable[float]) -> dict[str, float | None]:
    values = list(values)
    if not values:
        return {
            "min": None,
            "max": None,
            "mean": None,
            "mean_absolute": None,
            "rms": None,
            "standard_deviation": None,
            "peak_absolute": None,
        }
    return {
        "min": min(values),
        "max": max(values),
        "mean": fmean(values),
        "mean_absolute": fmean(abs(value) for value in values),
        "rms": math.sqrt(fmean(value * value for value in values)),
        "standard_deviation": pstdev(values),
        "peak_absolute": max(abs(value) for value in values),
    }


def _integrate_absolute(samples: list[VelocitySample], field: str) -> float:
    total = 0.0
    for previous, current in zip(samples, samples[1:]):
        dt = current.time - previous.time
        if dt > 0.0:
            previous_value = abs(float(getattr(previous, field)))
            current_value = abs(float(getattr(current, field)))
            total += 0.5 * (previous_value + current_value) * dt
    return total


class VelocitySeries:
    """Collect a velocity stream and calculate filtered finite differences."""

    def __init__(
        self,
        name: str,
        acceleration_filter_alpha: float = 0.25,
        calculate_jerk: bool = True,
    ):
        if not 0.0 < acceleration_filter_alpha <= 1.0:
            raise ValueError("acceleration_filter_alpha must be in (0, 1]")
        self.name = name
        self.calculate_jerk = calculate_jerk
        self.acceleration_filter_alpha = acceleration_filter_alpha
        self.samples: list[VelocitySample] = []

    def add(self, timestamp: float, linear: float, angular: float) -> VelocitySample:
        raw_linear_acceleration = None
        raw_angular_acceleration = None
        linear_acceleration = None
        angular_acceleration = None
        linear_jerk = None
        angular_jerk = None

        if self.samples:
            previous = self.samples[-1]
            dt = timestamp - previous.time
            if dt > 0.0:
                raw_linear_acceleration = (
                    linear - previous.linear_velocity
                ) / dt
                raw_angular_acceleration = (
                    angular - previous.angular_velocity
                ) / dt
                alpha = self.acceleration_filter_alpha
                if previous.linear_acceleration is None:
                    linear_acceleration = raw_linear_acceleration
                    angular_acceleration = raw_angular_acceleration
                else:
                    linear_acceleration = (
                        alpha * raw_linear_acceleration
                        + (1.0 - alpha) * previous.linear_acceleration
                    )
                    angular_acceleration = (
                        alpha * raw_angular_acceleration
                        + (1.0 - alpha) * previous.angular_acceleration
                    )
                if (
                    self.calculate_jerk
                    and previous.linear_acceleration is not None
                    and previous.angular_acceleration is not None
                ):
                    linear_jerk = (
                        linear_acceleration - previous.linear_acceleration
                    ) / dt
                    angular_jerk = (
                        angular_acceleration - previous.angular_acceleration
                    ) / dt

        sample = VelocitySample(
            time=timestamp,
            linear_velocity=linear,
            angular_velocity=angular,
            linear_acceleration=linear_acceleration,
            angular_acceleration=angular_acceleration,
            raw_linear_acceleration=raw_linear_acceleration,
            raw_angular_acceleration=raw_angular_acceleration,
            linear_jerk=linear_jerk,
            angular_jerk=angular_jerk,
        )
        self.samples.append(sample)
        return sample

    def summary(self) -> dict[str, object]:
        if not self.samples:
            return {"sample_count": 0}

        duration = self.samples[-1].time - self.samples[0].time
        sample_rate = (
            (len(self.samples) - 1) / duration
            if duration > 0.0 and len(self.samples) > 1
            else None
        )
        stopped_count = sum(
            abs(sample.linear_velocity) < 0.02
            and abs(sample.angular_velocity) < 0.02
            for sample in self.samples
        )
        linear_acceleration = [
            sample.linear_acceleration
            for sample in self.samples
            if sample.linear_acceleration is not None
        ]
        angular_acceleration = [
            sample.angular_acceleration
            for sample in self.samples
            if sample.angular_acceleration is not None
        ]
        summary = {
            "sample_count": len(self.samples),
            "duration_seconds": duration,
            "average_sample_rate_hz": sample_rate,
            "stationary_fraction": stopped_count / len(self.samples),
            "estimated_distance_travelled_m": _integrate_absolute(
                self.samples, "linear_velocity"
            ),
            "estimated_absolute_rotation_rad": _integrate_absolute(
                self.samples, "angular_velocity"
            ),
            "linear_velocity_mps": _signal_statistics(
                sample.linear_velocity for sample in self.samples
            ),
            "angular_velocity_radps": _signal_statistics(
                sample.angular_velocity for sample in self.samples
            ),
            "linear_acceleration_mps2": _signal_statistics(linear_acceleration),
            "angular_acceleration_radps2": _signal_statistics(
                angular_acceleration
            ),
        }
        if self.calculate_jerk:
            summary.update(
                {
                    "linear_jerk_mps3": _signal_statistics(
                        sample.linear_jerk
                        for sample in self.samples
                        if sample.linear_jerk is not None
                    ),
                    "angular_jerk_radps3": _signal_statistics(
                        sample.angular_jerk
                        for sample in self.samples
                        if sample.angular_jerk is not None
                    ),
                }
            )
        return summary


def tracking_summary(samples: list[TrackingSample]) -> dict[str, object]:
    """Summarize actual-minus-target velocity tracking errors."""

    if not samples:
        return {"sample_count": 0}
    return {
        "sample_count": len(samples),
        "linear_error_mps": _signal_statistics(
            sample.linear_error for sample in samples
        ),
        "angular_error_radps": _signal_statistics(
            sample.angular_error for sample in samples
        ),
    }


def value_for_plot(field: str, value: float) -> float:
    """Replace out-of-range acceleration values with a line-breaking NaN."""

    if (
        field in ACCELERATION_FIELDS
        and not ACCELERATION_PLOT_MIN <= value <= ACCELERATION_PLOT_MAX
    ):
        return math.nan
    return value


def robust_jerk_plot_limit(values: Iterable[float]) -> float | None:
    """Return a symmetric plot limit that ignores the largest two percent."""

    absolute_values = sorted(
        abs(value) for value in values if math.isfinite(value)
    )
    if not absolute_values:
        return None
    index = math.floor(
        (len(absolute_values) - 1) * JERK_PLOT_PERCENTILE / 100.0
    )
    return max(absolute_values[index], JERK_PLOT_MIN_ABS_LIMIT)


def jerk_value_for_plot(value: float, limit: float) -> float:
    """Replace jerk outside the robust plot range with a line-breaking NaN."""

    return value if abs(value) <= limit else math.nan


def should_plot_series(source: str, _field: str) -> bool:
    """Exclude noisy odometry from all motion-profile panels."""

    return source != "odometry"


class MotionReport:
    """Write reproducible CSV, JSON, and plot artifacts for a motion run."""

    COLORS = {
        "nav_command": "#1f77b4",
        "smoothed_command": "#ff7f0e",
        "odometry": "#2ca02c",
    }

    def __init__(
        self,
        output_directory: Path,
        series: dict[str, VelocitySeries],
        tracking_samples: list[TrackingSample],
        metadata: dict[str, object],
    ):
        self.output_directory = output_directory
        self.series = series
        self.tracking_samples = tracking_samples
        self.metadata = metadata

    def write(self, reason: str) -> list[Path]:
        """Write all report artifacts and return their paths."""

        self.output_directory.mkdir(parents=True, exist_ok=True)
        artifacts = [
            self._write_samples_csv(),
            self._write_tracking_csv(),
            self._write_summary(reason),
            self._write_velocity_plot(),
            self._write_jerk_plot(),
        ]
        if self.tracking_samples:
            artifacts.append(self._write_tracking_plot())
        return artifacts

    def _write_samples_csv(self) -> Path:
        path = self.output_directory / "velocity_samples.csv"
        temporary_path = path.with_suffix(".csv.tmp")
        fields = ["source", *VelocitySample.__dataclass_fields__.keys()]
        with temporary_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            for source, velocity_series in self.series.items():
                for sample in velocity_series.samples:
                    writer.writerow({"source": source, **asdict(sample)})
        temporary_path.replace(path)
        return path

    def _write_tracking_csv(self) -> Path:
        path = self.output_directory / "tracking_error_samples.csv"
        temporary_path = path.with_suffix(".csv.tmp")
        fields = [
            *TrackingSample.__dataclass_fields__.keys(),
            "linear_error",
            "angular_error",
        ]
        with temporary_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            for sample in self.tracking_samples:
                writer.writerow(
                    {
                        **asdict(sample),
                        "linear_error": sample.linear_error,
                        "angular_error": sample.angular_error,
                    }
                )
        temporary_path.replace(path)
        return path

    def _write_summary(self, reason: str) -> Path:
        path = self.output_directory / "summary.json"
        temporary_path = path.with_suffix(".json.tmp")
        report = {
            **self.metadata,
            "report_generated_at": datetime.now().astimezone().isoformat(),
            "report_reason": reason,
            "streams": {
                name: velocity_series.summary()
                for name, velocity_series in self.series.items()
            },
            "smoothed_command_tracking": tracking_summary(
                self.tracking_samples
            ),
        }
        temporary_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(path)
        return path

    def _write_velocity_plot(self) -> Path:
        from matplotlib.figure import Figure

        path = self.output_directory / "velocity_and_acceleration.png"
        temporary_path = path.with_suffix(".png.tmp")
        figure = Figure(figsize=(14, 15))
        axes = figure.subplots(4, 1, sharex=True)
        definitions = [
            ("linear_velocity", "Linear velocity (m/s)"),
            ("angular_velocity", "Angular velocity (rad/s)"),
            ("linear_acceleration", "Linear acceleration (m/s²)"),
            ("angular_acceleration", "Angular acceleration (rad/s²)"),
        ]
        for axis, (field, label) in zip(axes, definitions):
            for name, velocity_series in self.series.items():
                if not should_plot_series(name, field):
                    continue
                times = []
                values = []
                for sample in velocity_series.samples:
                    value = getattr(sample, field)
                    if value is not None:
                        times.append(sample.time)
                        values.append(value_for_plot(field, value))
                if times:
                    axis.plot(
                        times,
                        values,
                        label=name.replace("_", " ").title(),
                        color=self.COLORS.get(name),
                        linewidth=1.2,
                    )
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            if field in ACCELERATION_FIELDS:
                axis.set_ylim(
                    ACCELERATION_PLOT_MIN,
                    ACCELERATION_PLOT_MAX,
                )
            if axis.lines:
                axis.legend(loc="upper right")
        axes[-1].set_xlabel("Elapsed time (s)")
        figure.suptitle("Jackal Motion Profile")
        figure.tight_layout()
        figure.savefig(temporary_path, format="png", dpi=150)
        temporary_path.replace(path)
        return path

    def _write_jerk_plot(self) -> Path:
        from matplotlib.figure import Figure

        path = self.output_directory / "jerk_profile.png"
        temporary_path = path.with_suffix(".png.tmp")
        figure = Figure(figsize=(14, 8))
        axes = figure.subplots(2, 1, sharex=True)
        definitions = [
            ("linear_jerk", "Linear jerk (m/s³)"),
            ("angular_jerk", "Angular jerk (rad/s³)"),
        ]
        for axis, (field, label) in zip(axes, definitions):
            panel_values = []
            for name, velocity_series in self.series.items():
                if not should_plot_series(name, field):
                    continue
                panel_values.extend(
                    getattr(sample, field)
                    for sample in velocity_series.samples
                    if getattr(sample, field) is not None
                )
            plot_limit = robust_jerk_plot_limit(panel_values)
            omitted_count = 0
            has_data = False
            for name, velocity_series in self.series.items():
                if not should_plot_series(name, field):
                    continue
                times = []
                values = []
                for sample in velocity_series.samples:
                    value = getattr(sample, field)
                    if value is not None and plot_limit is not None:
                        times.append(sample.time)
                        plotted_value = jerk_value_for_plot(value, plot_limit)
                        values.append(plotted_value)
                        omitted_count += math.isnan(plotted_value)
                if times:
                    axis.plot(
                        times,
                        values,
                        label=name.replace("_", " ").title(),
                        color=self.COLORS.get(name),
                        linewidth=1.2,
                    )
                    has_data = True
            axis.axhline(0.0, color="black", linewidth=0.8)
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            if plot_limit is not None:
                axis.set_ylim(-plot_limit, plot_limit)
            if omitted_count:
                axis.text(
                    0.01,
                    0.97,
                    (
                        f"{omitted_count} spikes outside "
                        f"±{plot_limit:.3g} omitted"
                    ),
                    transform=axis.transAxes,
                    verticalalignment="top",
                    fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.8},
                )
            if has_data:
                axis.legend(loc="upper right")
        axes[-1].set_xlabel("Elapsed time (s)")
        figure.suptitle("Jackal Command Jerk Profile")
        figure.tight_layout()
        figure.savefig(temporary_path, format="png", dpi=150)
        temporary_path.replace(path)
        return path

    def _write_tracking_plot(self) -> Path:
        from matplotlib.figure import Figure

        path = self.output_directory / "velocity_tracking_error.png"
        temporary_path = path.with_suffix(".png.tmp")
        times = [sample.time for sample in self.tracking_samples]
        linear_errors = [
            sample.linear_error for sample in self.tracking_samples
        ]
        angular_errors = [
            sample.angular_error for sample in self.tracking_samples
        ]
        figure = Figure(figsize=(14, 8))
        axes = figure.subplots(2, 1, sharex=True)
        axes[0].plot(times, linear_errors, color="#d62728", linewidth=1.0)
        axes[0].set_ylabel("Linear error (m/s)")
        axes[1].plot(times, angular_errors, color="#9467bd", linewidth=1.0)
        axes[1].set_ylabel("Angular error (rad/s)")
        axes[1].set_xlabel("Elapsed time (s)")
        for axis in axes:
            axis.axhline(0.0, color="black", linewidth=0.8)
            axis.grid(True, alpha=0.3)
        figure.suptitle("Odometry Minus Smoothed Command")
        figure.tight_layout()
        figure.savefig(temporary_path, format="png", dpi=150)
        temporary_path.replace(path)
        return path
