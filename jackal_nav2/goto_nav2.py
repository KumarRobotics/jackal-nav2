"""Command-line client for sequential Nav2 waypoint execution."""

import argparse
import math
from pathlib import Path
import sys
from typing import List, Sequence, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml


Waypoint = Tuple[float, float, float]


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_waypoints_from_yaml(path: str) -> Tuple[str, List[Waypoint]]:
    waypoint_path = Path(path).expanduser()
    with waypoint_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError("Waypoint file must contain a YAML mapping")

    frame_id = str(data.get("frame_id", "map")).strip()
    if not frame_id:
        raise ValueError("frame_id cannot be empty")

    raw_waypoints = data.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("waypoints must be a non-empty YAML list")

    waypoints: List[Waypoint] = []
    for index, waypoint in enumerate(raw_waypoints):
        if isinstance(waypoint, dict):
            if "x" not in waypoint or "y" not in waypoint:
                raise ValueError(f"Waypoint {index} must contain x and y")
            x = float(waypoint["x"])
            y = float(waypoint["y"])
            yaw = float(waypoint.get("yaw", 0.0))
        elif isinstance(waypoint, (list, tuple)) and len(waypoint) >= 2:
            x = float(waypoint[0])
            y = float(waypoint[1])
            yaw = float(waypoint[2]) if len(waypoint) > 2 else 0.0
        else:
            raise ValueError(f"Invalid waypoint at index {index}: {waypoint!r}")
        waypoints.append((x, y, yaw))

    return frame_id, waypoints


class Nav2GotoPoseClient(Node):
    def __init__(
        self,
        action_name: str = "navigate_to_pose",
        namespace: str = "",
        server_timeout: float = 30.0,
    ):
        super().__init__("nav2_goto_pose_client", namespace=namespace)
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._server_timeout = server_timeout
        self._goal_handle = None

    def wait_for_server(self) -> bool:
        self.get_logger().info(
            f"Waiting up to {self._server_timeout:.1f}s for Nav2 action server"
        )
        return self._client.wait_for_server(timeout_sec=self._server_timeout)

    def build_goal(
        self, x: float, y: float, yaw: float, frame_id: str
    ) -> NavigateToPose.Goal:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def run_waypoints(
        self, waypoints: Sequence[Waypoint], frame_id: str = "map"
    ) -> bool:
        if not self.wait_for_server():
            self.get_logger().error("Nav2 action server was not available")
            return False

        for index, (x, y, yaw) in enumerate(waypoints, start=1):
            self.get_logger().info(
                f"Waypoint {index}/{len(waypoints)}: "
                f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, frame={frame_id}"
            )
            if not self.send_goal_and_wait(x, y, yaw, frame_id):
                self.get_logger().error(f"Waypoint {index} failed; stopping")
                return False
        return True

    def send_goal_and_wait(
        self, x: float, y: float, yaw: float, frame_id: str
    ) -> bool:
        send_future = self._client.send_goal_async(
            self.build_goal(x, y, yaw, frame_id),
            feedback_callback=self.feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()

        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error("Goal was rejected")
            self._goal_handle = None
            return False

        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        self._goal_handle = None

        if wrapped_result is None:
            self.get_logger().error("Goal returned no result")
            return False
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded")
            return True

        self.get_logger().error(
            f"Navigation failed with action status {wrapped_result.status}"
        )
        return False

    def feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f} m",
            throttle_duration_sec=5.0,
        )

    def cancel_active_goal(self) -> None:
        if self._goal_handle is None:
            return
        self.get_logger().warn("Cancelling active Nav2 goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        self._goal_handle = None


def parse_arguments(arguments: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send YAML waypoints sequentially to Nav2"
    )
    parser.add_argument("waypoint_file")
    parser.add_argument("--namespace", default="")
    parser.add_argument("--action-name", default="navigate_to_pose")
    parser.add_argument("--server-timeout", type=float, default=30.0)
    return parser.parse_args(arguments)


def main(args=None):
    cli_args = remove_ros_args(args=sys.argv if args is None else args)[1:]
    parsed = parse_arguments(cli_args)
    frame_id, waypoints = load_waypoints_from_yaml(parsed.waypoint_file)

    rclpy.init(args=args)
    node = Nav2GotoPoseClient(
        action_name=parsed.action_name,
        namespace=parsed.namespace,
        server_timeout=parsed.server_timeout,
    )
    success = False
    try:
        success = node.run_waypoints(waypoints, frame_id)
    except KeyboardInterrupt:
        node.cancel_active_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
