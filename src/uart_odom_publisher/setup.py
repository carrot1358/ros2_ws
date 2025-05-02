from setuptools import setup
import os
from glob import glob

package_name = 'uart_odom_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carrot',
    maintainer_email='carrot1358@gmail.com',
    description='UART to Odometry Publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_odom_node = uart_odom_publisher.uart_odom_node:main',
            'teleop_node = uart_odom_publisher.teleop_node:main',
            'teleop_joy = uart_odom_publisher.teleop_joy:main',
        ],
    },
)
