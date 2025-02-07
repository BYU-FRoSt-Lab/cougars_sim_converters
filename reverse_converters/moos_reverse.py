import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
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
        modem_status_heading = int(10 * -msg.vector.z) # just negate to put it into modem imu frame, also ModemStatus yaw is scaled down by 10, and it needs to be int
        converted_heading_msg.attitude_yaw = modem_status_heading
        print(converted_heading_msg.attitude_yaw )
        self.ModemStatus_publisher.publish(converted_heading_msg)

    

 
def main(args=None):
    rclpy.init(args=args)
    node = MOOSReverse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()