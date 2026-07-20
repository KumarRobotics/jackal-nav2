"""Convert Nav2 physical velocity commands to the Jackal Joy interface."""

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class CmdVelToJoy(Node):
    """Bridge stamped Nav2 commands to calibrated, normalized Joy axes."""

    def __init__(self):
        super().__init__("cmd_vel_to_joy")

        self.declare_parameter("linear_speed_at_full_axis", 2.0)
        self.declare_parameter("angular_speed_at_full_axis", 0.75)
        self.declare_parameter("cmd_vel_topic", "cmd_vel_smoothed")
        self.declare_parameter("joy_input_topic", "joy")
        self.declare_parameter("joy_output_topic", "joy")
        self.declare_parameter("publish_frequency", 50.0)
        self.declare_parameter("manual_override_axis", 4)
        self.declare_parameter("manual_override_value", -1.0)

        self.linear_speed_at_full_axis = float(
            self.get_parameter("linear_speed_at_full_axis").value
        )
        self.angular_speed_at_full_axis = float(
            self.get_parameter("angular_speed_at_full_axis").value
        )
        publish_frequency = float(self.get_parameter("publish_frequency").value)
        self.manual_override_axis = int(
            self.get_parameter("manual_override_axis").value
        )
        self.manual_override_value = float(
            self.get_parameter("manual_override_value").value
        )

        if self.linear_speed_at_full_axis <= 0.0:
            raise ValueError("linear_speed_at_full_axis must be positive")
        if self.angular_speed_at_full_axis <= 0.0:
            raise ValueError("angular_speed_at_full_axis must be positive")
        if publish_frequency <= 0.0:
            raise ValueError("publish_frequency must be positive")
        if self.manual_override_axis < 0:
            raise ValueError("manual_override_axis cannot be negative")

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        joy_input_topic = str(self.get_parameter("joy_input_topic").value)
        joy_output_topic = str(self.get_parameter("joy_output_topic").value)

        self.block_autonomy = False
        self.last_cmd = TwistStamped()

        self.cmd_sub = self.create_subscription(
            TwistStamped, cmd_vel_topic, self.cmd_cb, 20
        )
        self.joy_sub = self.create_subscription(
            Joy, joy_input_topic, self.joy_cb, 20
        )
        self.joy_pub = self.create_publisher(Joy, joy_output_topic, 1)
        self.timer = self.create_timer(1.0 / publish_frequency, self.publish_joy)

        self.get_logger().info(
            "CmdVel to Joy bridge started: "
            f"input={cmd_vel_topic}, output={joy_output_topic}, "
            f"full-scale linear={self.linear_speed_at_full_axis:.2f} m/s, "
            f"angular={self.angular_speed_at_full_axis:.2f} rad/s"
        )

    @staticmethod
    def clamp(value: float, minimum: float = -1.0, maximum: float = 1.0) -> float:
        return max(min(value, maximum), minimum)

    @classmethod
    def velocity_to_axis(cls, velocity: float, full_axis_speed: float) -> float:
        if full_axis_speed <= 0.0:
            raise ValueError("full_axis_speed must be positive")
        return cls.clamp(velocity / full_axis_speed)

    def joy_cb(self, msg: Joy) -> None:
        if len(msg.axes) > self.manual_override_axis:
            self.block_autonomy = (
                abs(
                    msg.axes[self.manual_override_axis]
                    - self.manual_override_value
                )
                < 1.0e-6
            )

    def cmd_cb(self, msg: TwistStamped) -> None:
        self.last_cmd = msg

    def publish_joy(self) -> None:
        if self.block_autonomy:
            return

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 8
        joy.axes[3] = self.velocity_to_axis(
            self.last_cmd.twist.linear.x, self.linear_speed_at_full_axis
        )
        joy.axes[2] = self.velocity_to_axis(
            self.last_cmd.twist.angular.z, self.angular_speed_at_full_axis
        )
        self.joy_pub.publish(joy)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
