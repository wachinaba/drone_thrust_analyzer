import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dynamixel_linear_slider'

# build a list of the data files
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append(("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files

data_files = package_files('launch/', data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wachinaba',
    maintainer_email='wachinaba@gmail.com',
    description='Dynamixel linear slider velocity control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
            entry_points={
            'console_scripts': [
                'velocity_publisher = dynamixel_linear_slider.velocity_publisher:main',
                'velocity_sine_publisher = dynamixel_linear_slider.velocity_sine_publisher:main',
                'dynamixel_handler_controller = dynamixel_linear_slider.dynamixel_handler_controller:main',
                'dynamixel_handler_odometry_node = dynamixel_linear_slider.dynamixel_handler_odometry_node:main',
                'dynamixel_driver_node = dynamixel_linear_slider.dynamixel_driver_node:main',
                'dynamixel_odometry_node = dynamixel_linear_slider.dynamixel_odometry_node:main',
                'sensor_fusion_node = dynamixel_linear_slider.sensor_fusion_node:main',
                'trajectory_generator_node = dynamixel_linear_slider.trajectory_generator_node:main',
                'position_controller_node = dynamixel_linear_slider.position_controller_node:main',
                'pose_to_position_converter_node = dynamixel_linear_slider.pose_to_position_converter_node:main',
                'velocity_command_converter_node = dynamixel_linear_slider.velocity_command_converter_node:main',
                'dynamixel_simulator_node = dynamixel_linear_slider.dynamixel_simulator_node:main',
                'simulator_test_publisher = dynamixel_linear_slider.simulator_test_publisher:main',
                'dynamixel_handler_position_controller_node = dynamixel_linear_slider.dynamixel_handler_position_controller_node:main',
                'position_test_publisher_node = dynamixel_linear_slider.position_test_publisher_node:main',
                'position_command_and_wait_node = dynamixel_linear_slider.position_command_and_wait_node:main',
            ],
        },
)
