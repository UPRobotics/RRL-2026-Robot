from setuptools import setup

package_name = 'magnetometer_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/magnetometer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboCup Team',
    maintainer_email='robotica@robocup.com',
    description='ROS2 magnetometer reader and visualizer',
    license='MIT',
    entry_points={
        'console_scripts': [
            'magnetometer_sender = magnetometer_pkg.magnetometer_sender:main',
            'magnetometer_receiver = magnetometer_pkg.magnetometer_receiver:main',
        ],
    },
)
