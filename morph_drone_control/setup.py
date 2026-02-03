import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'morph_drone_control'

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
data_files = package_files('config/', data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wachinaba',
    maintainer_email='wachinaba@gmail.com',
    description='Morph drone Dynamixel motor control package with matplotlib GUI slider',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'matplotlib_slider_node = morph_drone_control.matplotlib_slider_node:main',
            'morph_drone_preview_node = morph_drone_control.morph_drone_preview_node:main',
        ],
    },
)


