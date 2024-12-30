## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    base = Path(get_package_share_directory('reverse_converters'))
    params_file = base / 'config' / 'config.yaml'

    # List contents of the directory to debug
    
    vehicle_namespace = 'coug0'

    depth = launch_ros.actions.Node(
        name='depth_reverse',
        package='reverse_converters',
        executable='depth_reverse',  
        namespace=vehicle_namespace,
        output='screen',
        parameters=[params_file]  
    )

    gps = launch_ros.actions.Node(
        name='gps_reverse',
        package='reverse_converters',
        executable='gps_reverse',  
        namespace=vehicle_namespace,
        output='screen',
        parameters=[params_file]  
    )

    dvl = launch_ros.actions.Node(
        name='dvl_reverse',
        package='reverse_converters',
        executable='dvl_reverse',  
        namespace=vehicle_namespace,
        output='screen',
        parameters=[params_file]  
    )

    return LaunchDescription([
        depth,
        # dvl,
        gps,
    ])


