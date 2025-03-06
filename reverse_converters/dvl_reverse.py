import rclpy
import tf_transformations
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3, PoseWithCovarianceStamped
from holoocean_interfaces.msg import DVLSensorRange
from nav_msgs.msg import Odometry
from dvl_msgs.msg import DVL, DVLDR


class DVLReverse(Node):

    def __init__(self):
        super().__init__('reverse_dvl')
        self.DVL_publisher_ = self.create_publisher(DVL, 'dvl/data', 10)
        self.DVLDR_publisher_ = self.create_publisher(DVLDR, 'dvl/position', 10)
        timer_period = 0.5  # seconds
        self.DVLVelocity_subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            '/holoocean/DVLSensorVelocity',
            self.Vel_callback,
            10)
        self.DVLRange_subscription = self.create_subscription(
            DVLSensorRange,
            '/holoocean/DVLSensorRange',
            self.range_callback,
            10)
        
        self.DVLdead_reckon_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/holoocean/dead_reckon',
            self.DR_callback,
            10)
        self.altitude = -1.0
        
 
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
        position_vector.y = -msg.pose.pose.position.y
        position_vector.z = -msg.pose.pose.position.z
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



    def Vel_callback(self, msg):
        # msg = TwistWithCovarianceStamped()
        publish_msg = DVL()
        publish_msg.header = msg.header

        publish_msg.velocity.x = msg.twist.twist.linear.x
        publish_msg.velocity.y = -msg.twist.twist.linear.y
        publish_msg.velocity.z = -msg.twist.twist.linear.z
        publish_msg.altitude = self.altitude
        # DVL needed variables: float64 altitude,geometry_msgs/Vector3 velocity

        publish_msg.velocity_valid = True

        publish_msg.covariance = [0.0] * 36

        self.DVL_publisher_.publish(publish_msg)
        # self.get_logger().info('Velocity: "%s"' % publish_msg.velocity)
        # self.get_logger().info('Altitude: "%s"' % publish_msg.altitude)

    def range_callback(self, msg):
        avg_altitude = 0.0
        for range_measure in msg.range:
            avg_altitude += range_measure
        
        avg_altitude = avg_altitude/len(msg.range)
        
        self.altitude = float(avg_altitude)
 
def main(args=None):
    rclpy.init(args=args)

    node = DVLReverse()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()