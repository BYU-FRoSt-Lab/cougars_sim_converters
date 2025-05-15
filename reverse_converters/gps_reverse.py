#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
import math

EARTH_RADIUS_METERS = 6371000

class OdomToNavSatFix(Node):
    '''
    :author: [Your Name]
    :date: December 2024

    A simple ROS2 node that subscribes to the gps_odom topic and converts the Odometry data to GPSFix messages.
    The Odometry data is converted from local Cartesian coordinates to latitude, longitude, and altitude.

    Subscribes:
        - /gps_odom (nav_msgs/msg/Odometry)
    Publishes:
        - extended_fix (gps_msgs/msg/GPSFix)
    '''
    def __init__(self):
        '''
        Creates a new OdomToNavSatFix node.
        '''
        super().__init__('gps_fix')
        
        # Declare parameters for the origin (datum)
        self.declare_parameter('origin.latitude', 34.0219)
        self.declare_parameter('origin.longitude', -118.4814)
        self.declare_parameter('origin.altitude', 0.0)
        self.declare_parameter('holoocean_vehicle', 'auv0')
        holoocean_vehicle = self.get_parameter('holoocean_vehicle').get_parameter_value().string_value

        self.declare_parameter('frost_vehicle', 'coug1')
        frost_vehicle = self.get_parameter('frost_vehicle').get_parameter_value().string_value
        
        # Subscribe to Odometry
        self.subscriber = self.create_subscription(
            Odometry,
            '/holoocean/' + holoocean_vehicle + '/GPSSensor',
            self.odom_callback,
            10
        )
        
        self.last_msg = None

        # Publisher for GPSFix
        self.publisher = self.create_publisher(GPSFix, frost_vehicle + '/extended_fix', 10)
    
    def odom_callback(self, msg: Odometry):
        '''
        Callback function for the Odometry subscription.
        Converts the Odometry data to GPSFix messages and publishes them.
        
        :param msg: The Odometry message received from the gps_odom topic.
        '''
        # Convert local Cartesian coordinates to latitude/longitude
        lat, lon = self.calculate_inverse_haversine(
            self.get_parameter('origin.latitude').get_parameter_value().double_value,
            self.get_parameter('origin.longitude').get_parameter_value().double_value,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Calculate altitude
        alt = msg.pose.pose.position.z + self.get_parameter('origin.altitude').get_parameter_value().double_value

        # Fill in the GPSFix message
        gps_fix = GPSFix()
        gps_fix.header = msg.header
        gps_fix.header.frame_id = "gps"
        gps_fix.latitude = lat
        gps_fix.longitude = lon
        gps_fix.altitude = alt
        gps_fix.status.satellites_used = 10

        # Set the covariance values
        gps_fix.position_covariance[0] = msg.pose.covariance[0]  # xx
        gps_fix.position_covariance[4] = msg.pose.covariance[7]  # yy
        gps_fix.position_covariance[8] = msg.pose.covariance[14]  # zz
        gps_fix.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN

        # Publish the GPSFix message
        self.last_msg = gps_fix
        self.publisher.publish(gps_fix)

    def calculate_inverse_haversine(self, ref_lat, ref_lon, x, y):
        # Convert reference point to radians
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        # Calculate distance and bearing
        d = math.sqrt(x**2 + y**2)
        theta = math.atan2(x, y)

        # Calculate new latitude and longitude
        lat_rad = math.asin(math.sin(ref_lat_rad) * math.cos(d / EARTH_RADIUS_METERS) +
                            math.cos(ref_lat_rad) * math.sin(d / EARTH_RADIUS_METERS) * math.cos(theta))
        lon_rad = ref_lon_rad + math.atan2(math.sin(theta) * math.sin(d / EARTH_RADIUS_METERS) * math.cos(ref_lat_rad),
                                           math.cos(d / EARTH_RADIUS_METERS) - math.sin(ref_lat_rad) * math.sin(lat_rad))

        # Convert back to degrees
        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)

        return lat, lon

def main(args=None):
    rclpy.init(args=args)
    node = OdomToNavSatFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
