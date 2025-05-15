#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCombiner(Node):
    def __init__(self):
        super().__init__('imu_combiner')
        
        # Initialize latest message storage
        self.dynamics_msg = None
        self.imu_msg = None

        self.declare_parameter('holoocean_vehicle', 'auv0')
        holoocean_vehicle = self.get_parameter('holoocean_vehicle').get_parameter_value().string_value

        self.declare_parameter('frost_vehicle', 'coug1')
        frost_vehicle = self.get_parameter('frost_vehicle').get_parameter_value().string_value
        
        # Create subscribers
        self.dynamics_sub = self.create_subscription(
            Imu,
            '/holoocean/' + holoocean_vehicle + '/DynamicsSensorIMU',
            self.dynamics_callback,
            10)
            
        self.imu_sub = self.create_subscription(
            Imu,
            '/holoocean/' + holoocean_vehicle + '/IMUSensor',
            self.imu_callback,
            10)
            
        # Create publisher
        self.publisher = self.create_publisher(Imu, frost_vehicle + '/modem_imu', 10)
        
        # Create timer to check and publish combined data at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def dynamics_callback(self, msg):
        self.dynamics_msg = msg

    def imu_callback(self, msg):
        self.imu_msg = msg

    def timer_callback(self):
        if self.dynamics_msg and self.imu_msg:
            combined_msg = Imu()
            
            # Copy header from IMU sensor
            combined_msg.header = self.imu_msg.header
            
            # Use orientation from DynamicsSensor
            combined_msg.orientation = self.dynamics_msg.orientation
            combined_msg.orientation_covariance = self.dynamics_msg.orientation_covariance
            
            # Copy all other fields from IMUSensor
            combined_msg.angular_velocity = self.imu_msg.angular_velocity
            combined_msg.linear_acceleration = self.imu_msg.linear_acceleration
            combined_msg.angular_velocity_covariance = self.imu_msg.angular_velocity_covariance
            combined_msg.linear_acceleration_covariance = self.imu_msg.linear_acceleration_covariance
            
            self.publisher.publish(combined_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCombiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
