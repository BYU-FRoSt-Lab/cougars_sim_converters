import rclpy
import tf_transformations
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVL, DVLDR
from holoocean_interfaces.msg import DVLSensorRange


class DVLReverse(Node):

    def __init__(self):
        super().__init__('reverse_dvl')

        self.declare_parameter('holoocean_vehicle', 'auv0')
        holoocean_vehicle = self.get_parameter('holoocean_vehicle').get_parameter_value().string_value

        self.declare_parameter('frost_vehicle', 'coug1')
        frost_vehicle = self.get_parameter('frost_vehicle').get_parameter_value().string_value

        self.DVL_publisher_ = self.create_publisher(DVL, frost_vehicle + '/dvl/data', 10)
        self.DVLDR_publisher_ = self.create_publisher(DVLDR, frost_vehicle + '/dvl/position', 10)

        timer_period = 0.5  # seconds
        self.DVLVelocity_subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            '/holoocean/' + holoocean_vehicle + '/DVLSensorVelocity',
            self.Vel_callback,
            10)
        
        # TODO fix this
        self.DVLdead_reckon_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/holoocean/dead_reckon',
            self.DR_callback,
            10)
        
        self.dvl_range_sub = self.create_subscription(
            DVLSensorRange,
            '/holoocean/' + holoocean_vehicle + '/DVLSensorRange',
            self.altitude_callback,
            10)
        
        self.altitude = 0.0
        
 
    def DR_callback(self, msg):
        publish_msg = DVLDR()
        publish_msg.header = msg.header
        # msg = Odometry()
        # publish_msg.position = msg.pose.pose.position
        # publish_msg.pos_std = msg.   //TODO: figure out where to get this data from
        
        # Convert quaternion to Euler angles (is this correct?)
        # order of the angles is z,y,x
        # uses r the intrinsic rotation 
        position_vector = Vector3()
        position_vector.x = msg.pose.pose.position.x
        position_vector.y = msg.pose.pose.position.y
        position_vector.z = msg.pose.pose.position.z
        publish_msg.position = position_vector

        orientation_q = msg.pose.pose.orientation
        orientation_list = [-orientation_q.x, orientation_q.y, orientation_q.z, -orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list, axes='rzyx')

        publish_msg.roll = roll
        publish_msg.pitch = pitch
        publish_msg.yaw = yaw

        self.DVLDR_publisher_.publish(publish_msg)

        # self.get_logger().info('Position: "%s"' % str(publish_msg))
        # self.get_logger().info('Roll: "%s"' % publish_msg.roll)
        # self.get_logger().info('Pitch: "%s"' % publish_msg.pitch)
        # self.get_logger().info('Yaw: "%s"' % publish_msg.yaw)


    def altitude_callback(self, msg: DVLSensorRange):
        self.altitude = float(sum(msg.range) / len(msg.range))
    
    
    def Vel_callback(self, msg):
        # msg = TwistWithCovarianceStamped()
        publish_msg = DVL()
        publish_msg.header = msg.header

        publish_msg.velocity = msg.twist.twist.linear


        publish_msg.altitude = self.altitude
        # DVL needed variables: float64 altitude,geometry_msgs/Vector3 velocity

        self.DVL_publisher_.publish(publish_msg)
        # self.get_logger().info('Velocity: "%s"' % publish_msg.velocity)
        # self.get_logger().info('Altitude: "%s"' % publish_msg.altitude)

 
def main(args=None):
    rclpy.init(args=args)

    node = DVLReverse()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()