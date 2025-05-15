import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from holoocean_interfaces.msg import ControlCommand
from frost_interfaces.msg import UCommand
import math


class UCommandBridge(Node):
    def __init__(self):
        super().__init__('ucommand_bridge')

        # Declare parameters
        self.declare_parameter('frost_vehicle', 'coug1')
        self.declare_parameter('holoocean_vehicle', 'auv0')
        self.declare_parameter('fin_scalar', 1.0)
        self.declare_parameter('publish_thruster', False)

        # Get parameter values
        frost_vehicle = self.get_parameter('frost_vehicle').get_parameter_value().string_value
        holoocean_vehicle = self.get_parameter('holoocean_vehicle').get_parameter_value().string_value
        self.fin_scalar = self.get_parameter('fin_scalar').get_parameter_value().double_value
        self.publish_thruster = self.get_parameter('publish_thruster').get_parameter_value().bool_value

        # Construct topic names from parameters
        frost_topic = f'/{frost_vehicle}/controls/command'
        holoocean_topic = f'/holoocean/{holoocean_vehicle}/ControlCommand'

        # Subscriptions
        self.frost_sub = self.create_subscription(
            UCommand,
            frost_topic,
            self.frost_callback,
            10
        )

        self.holoocean_sub = self.create_subscription(
            ControlCommand,
            holoocean_topic,
            self.holoocean_callback,
            10
        )

        # Publishers
        self.frost_pub = self.create_publisher(
            UCommand,
            frost_topic,
            10
        )

        self.holoocean_pub = self.create_publisher(
            ControlCommand,
            "/holoocean/command/control",
            10
        )

    def frost_callback(self, msg: UCommand):
        # If this was sent from the bridge ignore the passing back to holoocean
        if msg.header.frame_id == "holoocean_to_frost":
            return

        # Convert fin angles from degrees to radians
        fins_rad = [-1 * math.radians(angle) for angle in msg.fin[:3]]

        # Thruster percentage directly used
        thruster = float(msg.thruster)

        # Pack into CommandControl message
        control_msg = ControlCommand()
        control_msg.header = Header()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = self.get_parameter('holoocean_vehicle').get_parameter_value().string_value
        control_msg.cs = fins_rad + [thruster]

        self.holoocean_pub.publish(control_msg)
        # self.get_logger().info("Forwarded to holoocean control/command")

    def holoocean_callback(self, msg: ControlCommand):
        if len(msg.cs) < 3:
            self.get_logger().warn("ControlCommand message has fewer than 3 fins")
            return

        # Convert fin angles from radians to degrees and apply scalar
        fins_deg = [math.degrees(angle) * self.fin_scalar for angle in msg.cs[:3]]

        u_cmd_msg = UCommand()
        u_cmd_msg.header = Header()
        u_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        u_cmd_msg.header.frame_id = "holoocean_to_frost"
        u_cmd_msg.fin = fins_deg + [0.0]  # Pad with unused value

        # Thruster: only publish if enabled
        u_cmd_msg.thruster = int(msg.cs[3]) if (self.publish_thruster and len(msg.cs) >= 4) else 0

        self.frost_pub.publish(u_cmd_msg)
        # self.get_logger().info("Forwarded to frost UCommand")


def main(args=None):
    rclpy.init(args=args)
    node = UCommandBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
