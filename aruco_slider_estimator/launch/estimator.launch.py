#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """ARマーカー式スライダ位置推定システムのlaunchファイル"""
    
    # Launch引数の定義
    marker_dictionary_arg = DeclareLaunchArgument(
        'marker_dictionary',
        default_value='DICT_4X4_100',
        description='使用するArUcoマーカーの辞書名'
    )
    
    base_board_config_arg = DeclareLaunchArgument(
        'base_board_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('aruco_slider_estimator'),
            'config',
            'base_board_config.json'
        ]),
        description='ベース側マーカー群の定義ファイルへのパス'
    )
    
    slider_board_config_arg = DeclareLaunchArgument(
        'slider_board_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('aruco_slider_estimator'),
            'config',
            'slider_board_config.json'
        ]),
        description='スライダ側マーカー群の定義ファイルへのパス'
    )
    
    detector_params_file_arg = DeclareLaunchArgument(
        'detector_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aruco_slider_estimator'),
            'config',
            'detector_params.yaml'
        ]),
        description='ArUco検出器のパラメータファイルへのパス'
    )
    
    world_frame_id_arg = DeclareLaunchArgument(
        'world_frame_id',
        default_value='world',
        description='ワールド座標系のフレーム名'
    )
    
    slider_frame_id_arg = DeclareLaunchArgument(
        'slider_frame_id',
        default_value='slider_base',
        description='スライダ座標系のフレーム名'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='/tfトピックをパブリッシュするかどうかのフラグ'
    )
    
    show_debug_image_arg = DeclareLaunchArgument(
        'show_debug_image',
        default_value='true',
        description='デバッグ用画像をパブリッシュするかどうかのフラグ'
    )
    
    # ノードの定義
    estimator_node = Node(
        package='aruco_slider_estimator',
        executable='estimator_node',
        name='estimator_node',
        output='screen',
        parameters=[{
            'marker_dictionary': LaunchConfiguration('marker_dictionary'),
            'base_board_config': LaunchConfiguration('base_board_config'),
            'slider_board_config': LaunchConfiguration('slider_board_config'),
            'detector_params_file': LaunchConfiguration('detector_params_file'),
            'world_frame_id': LaunchConfiguration('world_frame_id'),
            'slider_frame_id': LaunchConfiguration('slider_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'show_debug_image': LaunchConfiguration('show_debug_image'),
        }],
        remappings=[
            ('~/input/image_raw', '/camera/image_raw'),
            ('~/input/camera_info', '/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        marker_dictionary_arg,
        base_board_config_arg,
        slider_board_config_arg,
        detector_params_file_arg,
        world_frame_id_arg,
        slider_frame_id_arg,
        publish_tf_arg,
        show_debug_image_arg,
        estimator_node,
    ]) 