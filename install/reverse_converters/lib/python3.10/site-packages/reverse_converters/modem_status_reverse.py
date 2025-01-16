import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVL, DVLDR
from seatrac_interfaces.msg import ModemStatus


class ModemMOOSReverse(Node):

    def __init__(self):
        super().__init__('reverse_modem_status')
        self.ModemStatus_publisher = self.create_publisher(ModemStatus, 'modem_status', 10)
        self.sim_heading_subscription = self.create_subscription(
            Vector3Stamped,
            '/holoocean/RotationSensor',
            self.heading_callback,
            10)
        
 
    def heading_callback(self, msg):
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

    node = ModemMOOSReverse()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()