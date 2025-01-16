import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVL, DVLDR
from seatrac_interfaces.msg import ModemStatus

# this node basically allows us to just have a single moos_bridge node so whether we are running in sim or not we can just launch the moos_bridge.cpp


class MOOSReverse(Node):

    def __init__(self):
        super().__init__('moos_reverse')

        # converted values to be published
        self.ModemStatus_publisher = self.create_publisher(ModemStatus, 'modem_status', 10)
        self.smoothed_output_publisher = self.create_publisher(Odometry, 'smoothed_output', 10)

        # subscriptions to be put
        self.sim_heading_subscription = self.create_subscription(
            Vector3Stamped,
            '/holoocean/RotationSensor',
            self.heading_converter_callback,
            10)
    
 
    def heading_converter_callback(self, msg):
        converted_heading_msg = ModemStatus()


        # process of converting into ModemStatus msg

        if msg.vector.z < 0.0:
            nav_heading = 360.0 + msg.vector.z
        else:
            nav_heading = msg.vector.z

        nav_heading = 360 - nav_heading

        # at this point, this is what MOOS wants, but we now change this to look like a modem_status message
        # since it is always positive, in the moos bridge it will always go to the second case.
        modem_status_heading = nav_heading * 10
        converted_heading_msg.attitude_yaw = modem_status_heading
        self.ModemStatus_publisher.publish(converted_heading_msg)

    

 
def main(args=None):
    rclpy.init(args=args)
    node = MOOSReverse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()