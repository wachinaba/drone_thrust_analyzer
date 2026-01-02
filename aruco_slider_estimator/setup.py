from setuptools import find_packages, setup

package_name = 'aruco_slider_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/base_board_config.json',
            'config/slider_board_config.json',
            'config/detector_params.yaml',
            'config/emeet_c960.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/estimator.launch.py',
            'launch/camera.launch.py',
            'launch/camera_with_info.launch.py',
        ]),
        ('share/' + package_name + '/scripts', [
            'scripts/generate_markers.py',
            'scripts/generate_board.py',
            'scripts/generate_all.py',
            'scripts/camera_calibration.py',
        ]),
        ('share/' + package_name + '/scripts/config', [
            'scripts/config/marker_generation.yaml',
        ]),
        ('share/' + package_name + '/scripts/config/templates', [
            'scripts/config/templates/base_board_template.yaml',
            'scripts/config/templates/slider_board_template.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wachinaba',
    maintainer_email='wachinaba@gmail.com',
    description='ARマーカー（ArUcoマーカー）を用いて直線運動するスライダの精密な位置を推定するROS 2パッケージ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estimator_node = aruco_slider_estimator.estimator_node:main',
        ],
    },
)
