import os
from glob import glob
from setuptools import setup

package_name = 'dynamixel_leadscrew_slider'

data_files = []
data_files.append((f'share/ament_index/resource_index/packages', [f'resource/{package_name}']))
data_files.append((f'share/{package_name}', ['package.xml']))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append((f'share/{package_name}/{path}', glob(path + '/**/*.*', recursive=True)))
    return data_files

for d in ['launch', 'config']:
    if os.path.isdir(d):
        data_files = package_files(d, data_files)

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wachinaba',
    maintainer_email='wachinaba@gmail.com',
    description='Leadscrew slider controller (XL-330) using DynamixelHandler-ros2. Current-control homing and mm commands.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leadscrew_slider_controller = dynamixel_leadscrew_slider.controller:main',
            'move_to_position = dynamixel_leadscrew_slider.move_to_position_node:main',
        ],
    },
)



