from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reverse_converters'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.[pxy][yma]*'))),   
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ue4',
    maintainer_email='ue4@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dvl_reverse = reverse_converters.dvl_reverse:main',
            'depth_reverse = reverse_converters.depth_reverse:main',
            'gps_reverse = reverse_converters.gps_reverse:main',
            'imu_reverse = reverse_converters.imu_reverse:main',
            'ucommand_bridge = reverse_converters.ucommand_bridge:main',
        ],
    },
)
