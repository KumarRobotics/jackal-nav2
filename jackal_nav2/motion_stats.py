"""ROS 2 node that records and reports Jackal motion quality statistics."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from jackal_nav2.motion_metrics import MotionReport, TrackingSample, VelocitySeries


def default_output_directory() -> Path:
    """Find the source repository's ignored plots directory when available."""

    module_path = Path(__file__).resolve()
    for parent in module_path.parents:
        if (parent / "package.xml").is_file() and (parent / "setup.py").is_file():
            return parent / "plots"

    # A normal colcon install copies this module under install/<package>.
    # Walk back to the workspace and prefer its source package when present.
    for parent in module_path.parents:
        source_package = parent / "src" / "jackal_nav2"
        if (source_package / "package.xml").is_file():
            return source_package / "plots"

    return Path.home() / ".ros" / "jackal_nav2" / "plots"


class MotionStats(Node):
    """Capture requested, smoothed, and measured Jackal motion."""

    def __init__(self):
        super().__init__("motion_stats")

        self.declare_parameter("nav_cmd_vel_topic", "cmd_vel_nav")
        self.declare_parameter("smoothed_cmd_vel_topic", "cmd_vel_smoothed")
        self.declare_parameter("odom_topic", "dlio/odom_node/odom")
        self.declare_parameter("output_directory", "")
        self.declare_parameter("report_interval", 30.0)
        self.declare_parameter("acceleration_filter_alpha", 0.25)

        nav_topic = str(self.get_parameter("nav_cmd_vel_topic").value)
        smoothed_topic = str(
            self.get_parameter("smoothed_cmd_vel_topic").value
        )
        odom_topic = str(self.get_parameter("odom_topic").value)
        output_parameter = str(
            self.get_parameter("output_directory").value
        ).strip()
        report_interval = float(
            self.get_parameter("report_interval").value
        )
        acceleration_filter_alpha = float(
            self.get_parameter("acceleration_filter_alpha").value
        )

        if report_interval <= 0.0:
            raise ValueError("report_interval must be positive")
        if not 0.0 < acceleration_filter_alpha <= 1.0:
            raise ValueError("acceleration_filter_alpha must be in (0, 1]")

        output_root = (
            Path(output_parameter).expanduser()
            if output_parameter
            else default_output_directory()
        )
        run_name = datetime.now().astimezone().strftime(
            "run_%Y%m%d_%H%M%S_%f"
        )
        self.output_directory = output_root / run_name
        self.output_directory.mkdir(parents=True, exist_ok=False)

        self.start_time = time.monotonic()
        self.series = {
            name: VelocitySeries(
                name,
                acceleration_filter_alpha,
                calculate_jerk=name != "odometry",
            )
            for name in ("nav_command", "smoothed_command", "odometry")
        }
        self.tracking_samples: list[TrackingSample] = []
        self.latest_smoothed_command: tuple[float, float] | None = None
        self._writing_report = False

        self.report = MotionReport(
            output_directory=self.output_directory,
            series=self.series,
            tracking_samples=self.tracking_samples,
            metadata={
                "started_at": datetime.now().astimezone().isoformat(),
                "topics": {
                    "nav_command": nav_topic,
                    "smoothed_command": smoothed_topic,
                    "odometry": odom_topic,
                },
                "acceleration_filter_alpha": acceleration_filter_alpha,
                "acceleration_method": (
                    "finite difference over receipt time followed by an "
                    "exponential moving average"
                ),
                "jerk_method": (
                    "finite difference of filtered acceleration over receipt "
                    "time for nav and smoothed commands only"
                ),
                "jerk_plot_policy": (
                    "symmetric 98th-percentile absolute limit with larger "
                    "values omitted from plots only"
                ),
                "tracking_error_convention": "odometry minus smoothed command",
            },
        )

        self.nav_subscription = self.create_subscription(
            TwistStamped,
            nav_topic,
            self._nav_command_callback,
            100,
        )
        self.smoothed_subscription = self.create_subscription(
            TwistStamped,
            smoothed_topic,
            self._smoothed_command_callback,
            100,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odometry_callback,
            qos_profile_sensor_data,
        )
        self.report_timer = self.create_timer(
            report_interval, self._periodic_report
        )

        self.get_logger().info(
            "Motion analysis started: "
            f"nav={nav_topic}, smoothed={smoothed_topic}, odom={odom_topic}; "
            f"reports={self.output_directory}"
        )

    def _elapsed_time(self) -> float:
        return time.monotonic() - self.start_time

    def _nav_command_callback(self, message: TwistStamped) -> None:
        self.series["nav_command"].add(
            self._elapsed_time(),
            message.twist.linear.x,
            message.twist.angular.z,
        )

    def _smoothed_command_callback(self, message: TwistStamped) -> None:
        linear = message.twist.linear.x
        angular = message.twist.angular.z
        self.series["smoothed_command"].add(
            self._elapsed_time(), linear, angular
        )
        self.latest_smoothed_command = (linear, angular)

    def _odometry_callback(self, message: Odometry) -> None:
        timestamp = self._elapsed_time()
        linear = message.twist.twist.linear.x
        angular = message.twist.twist.angular.z
        self.series["odometry"].add(timestamp, linear, angular)
        if self.latest_smoothed_command is not None:
            target_linear, target_angular = self.latest_smoothed_command
            self.tracking_samples.append(
                TrackingSample(
                    time=timestamp,
                    target_linear=target_linear,
                    actual_linear=linear,
                    target_angular=target_angular,
                    actual_angular=angular,
                )
            )

    def _periodic_report(self) -> None:
        self.write_report("periodic autosave")

    def write_report(self, reason: str) -> None:
        if self._writing_report:
            return
        if not any(series.samples for series in self.series.values()):
            return
        self._writing_report = True
        try:
            self.report.write(reason)
            if rclpy.ok():
                self.get_logger().info(
                    f"Motion analysis report updated in {self.output_directory}"
                )
        except Exception as error:  # Keep telemetry failures from stopping Nav2.
            if rclpy.ok():
                self.get_logger().error(f"Unable to write motion report: {error}")
        finally:
            self._writing_report = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionStats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_report("node shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
