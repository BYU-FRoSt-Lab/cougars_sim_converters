## Holocean launch file 
# Author: Braden Meyers

from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path

def generate_launch_description():
    print('Launching HoloOcean Vehicle Simulation')

    base = Path(get_package_share_directory('reverse_converters'))
    params_file = base / 'config' / 'config.yaml'

    # List contents of the directory to debug
    
    holoocean = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('holoocean_main'),
                'launch',
                'holoocean_launch.py'
            )
        ])
    )

    depth = launch_ros.actions.Node(
        name='depth_reverse',
        package='reverse_converters',
        executable='depth_reverse',  
        output='screen',
        parameters=[params_file]  
    )

    gps = launch_ros.actions.Node(
        name='gps_reverse',
        package='reverse_converters',
        executable='gps_reverse',  
        output='screen',
        parameters=[params_file]  
    )

    dvl = launch_ros.actions.Node(
        name='dvl_reverse',
        package='reverse_converters',
        executable='dvl_reverse',  
        output='screen',
        parameters=[params_file]  
    )

    imu = launch_ros.actions.Node(
        name='imu_reverse',
        package='reverse_converters',
        executable='imu_reverse',  
        output='screen',
        parameters=[params_file]  
    )

    ucommand = launch_ros.actions.Node(
        name='ucommand_bridge',
        package='reverse_converters',
        executable='ucommand_bridge',  
        output='screen',
        parameters=[params_file]  
    )

    return LaunchDescription([
        depth,
        dvl,
        gps,
        holoocean,
        imu,
        ucommand,
    ])


