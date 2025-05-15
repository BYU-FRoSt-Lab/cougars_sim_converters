import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import FluidPressure

GRAVITY = 9.81  # m/s^2
FLUID_DENSITY_BASE = 997  # kg/m^3

class PressureConverter(Node):
    def __init__(self):
        super().__init__('pressure_converter')

        # Declare parameters
        self.declare_parameter('water_salinity_ppt', 0.0)
        self.declare_parameter('fluid_pressure_atm', 87250.0)
        self.declare_parameter('holoocean_vehicle', 'auv0')
        self.declare_parameter('frost_vehicle', 'coug1')

        frost_vehicle = self.get_parameter('frost_vehicle').get_parameter_value().string_value
        holoocean_vehicle = self.get_parameter('holoocean_vehicle').get_parameter_value().string_value

        # Create publisher
        self.pressure_publisher = self.create_publisher(
            FluidPressure,
            frost_vehicle + '/pressure/data',
            10
        )

        # Create subscription
        self.depth_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/holoocean/' + holoocean_vehicle + '/DepthSensor',
            self.depth_callback,
            10
        )

    def depth_callback(self, depth_msg):
        pressure_msg = FluidPressure()
        pressure_msg.header = depth_msg.header

        water_salinity_ppt = self.get_parameter('water_salinity_ppt').value
        fluid_pressure_atm = self.get_parameter('fluid_pressure_atm').value

        # Convert depth to pressure
        depth = depth_msg.pose.pose.position.z
        pressure = (fluid_pressure_atm - 
                    (depth * (FLUID_DENSITY_BASE + water_salinity_ppt) * GRAVITY))

        pressure_msg.fluid_pressure = pressure
        self.pressure_publisher.publish(pressure_msg)

def main(args=None):
    rclpy.init(args=args)
    pressure_converter = PressureConverter()
    rclpy.spin(pressure_converter)
    pressure_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
