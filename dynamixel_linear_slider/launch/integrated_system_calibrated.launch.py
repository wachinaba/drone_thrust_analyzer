#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
    EmitEvent,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """AR Web クライアント + キャリブレーション統合制御版の統合システム launch"""

    # Launch 引数の宣言
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    slider_params_arg = DeclareLaunchArgument(
        "slider_params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("dynamixel_leadscrew_slider"),
                "config",
                "slider_params.yaml",
            ]
        ),
        description="Leadscrew slider params YAML",
    )

    enable_bottom_slider_arg = DeclareLaunchArgument(
        "enable_bottom_slider",
        default_value="true",
        description="Enable bottom leadscrew slider (slider_bottom namespace)",
    )

    enable_top_slider_arg = DeclareLaunchArgument(
        "enable_top_slider",
        default_value="true",
        description="Enable top leadscrew slider (slider_top namespace)",
    )

    enable_vertical_slider_arg = DeclareLaunchArgument(
        "enable_vertical_slider",
        default_value="true",
        description="Enable vertical leadscrew slider (slider_vertical namespace)",
    )

    # DynamixelHandler_ros2 ノード
    dynamixel_handler_node = Node(
        package="dynamixel_handler",
        executable="dynamixel_handler",
        name="dynamixel_handler",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "dynamixel_handler:=WARN"],
        parameters=[
            {
                "device_name": "/dev/ttyUSB0",
                "baudrate": 57600,
                "latency_timer": 16,
                "init/dummy_servo_list": [-1],
                "init/baudrate_auto_set": False,
                "init/uesd_servo_series": {"X": True, "P": False, "Pro": False},
                "init/expected_servo_num": 0,
                "init/servo_auto_search": {
                    "min_id": 0,
                    "max_id": 10,
                    "retry_times": 0,
                },
                "init/hardware_error_auto_clean": True,
                "init/torque_auto_enable": True,
                "term/torque_auto_disable": True,
                "term/servo_auto_stop": True,
                "default/profile_acc": 600.0,
                "default/profile_vel": 100.0,
                "default/return_delay_time": 0.0,
                "loop_rate": 100,
                "verbose_ratio": 300,
                "pub_outdated_present_value": True,
                "pub_ratio/present": {
                    "pwm": 2,
                    "current": 2,
                    "velocity": 2,
                    "position": 2,
                    "velocity_trajectory": 0,
                    "position_trajectory": 0,
                    "input_voltage": 29,
                    "temperature": 29,
                },
                "pub_ratio/status": 47,
                "pub_ratio/goal": 11,
                "pub_ratio/gain": 101,
                "pub_ratio/limit": 307,
                "pub_ratio/error": 53,
                "method/fast_read": False,
                "method/split_read": False,
                "method/split_write": True,
                "option/external_port": {
                    "use": False,
                    "pub_ratio/data": 10,
                    "pub_ratio/mode": 100,
                    "verbose/callback": False,
                    "verbose/write": False,
                    "verbose/read": {"raw": False, "err": False},
                },
                "max_log_width": 6,
                "verbose/callback": True,
                "verbose/write_status": False,
                "verbose/write_goal": False,
                "verbose/write_gain": False,
                "verbose/write_limit": False,
                "verbose/read_status": {"raw": False, "err": False},
                "verbose/read_present": {"raw": False, "err": False},
                "verbose/read_goal": {"raw": False, "err": False},
                "verbose/read_gain": {"raw": False, "err": False},
                "verbose/read_limit": {"raw": False, "err": False},
                "verbose/read_hardware_error": True,
                "dyn_comm/retry_num": 5,
                "dyn_comm/inerval_msec": 4,
                "dyn_comm/verbose": False,
                "debug": False,
                "no_use_command_line": False,
            }
        ],
    )

    # 力センサノード
    force_sensor_node = Node(
        package="force_sensor_pkg",
        executable="zef6a_sensor_driver_continuous",
        name="force_sensor_node",
        output="screen",
    )

    # AR マーカー Web クライアントノード（Windows 側 AR サーバから corners を取得）
    ar_marker_web_client_node = Node(
        package="dynamixel_linear_slider",
        executable="ar_marker_web_client_node",
        name="ar_marker_web_client_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "server_url": "http://localhost:8000/detect",
                "poll_frequency": 6.0,
                "base_board_config": PathJoinSubstitution(
                    [
                        FindPackageShare("aruco_slider_estimator"),
                        "config",
                        "base_board_config.json",
                    ]
                ),
                "slider_board_config": PathJoinSubstitution(
                    [
                        FindPackageShare("aruco_slider_estimator"),
                        "config",
                        "slider_board_config.json",
                    ]
                ),
                "camera_info_file": PathJoinSubstitution(
                    [
                        FindPackageShare("aruco_slider_estimator"),
                        "config",
                        "emeet_c960.yaml",
                    ]
                ),
                "position_axis": "x",
                "scale_factor": -1.0,
            }
        ],
    )

    # キャリブレーション統合スライダ制御ノード
    calibrated_slider_controller_node = Node(
        package="dynamixel_linear_slider",
        executable="calibrated_slider_controller_node",
        name="calibrated_slider_controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "motor_id": 1,
                "rack_pitch": 0.106214,
                "gear_ratio": 1.0,
                "control_frequency": 100.0,
                "profile_velocity_deg_s": 2000.0,
                "profile_accel_deg_ss": 200.0,
                "calibration_file": PathJoinSubstitution(
                    [
                        FindPackageShare("dynamixel_linear_slider"),
                        "config",
                        "calibration.yaml",
                    ]
                ),
            }
        ],
    )

    # Leadscrew スライダ・コントローラ（dynamixel_leadscrew_slider）- 2 台構成
    leadscrew_slider_controller_bottom_node = Node(
        package="dynamixel_leadscrew_slider",
        executable="leadscrew_slider_controller",
        name="leadscrew_slider_controller",
        namespace="slider_bottom",
        condition=IfCondition(LaunchConfiguration("enable_bottom_slider")),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("slider_params_file"),
            {
                "motor.id": 3,
                "homing.min_soft_limit_range_mm": 240.0,
            },
        ],
        remappings=[
            ("/move_mm", "/slider_bottom/move_mm"),
            ("/current_position", "/slider_bottom/current_position"),
        ],
    )

    leadscrew_slider_controller_top_node = Node(
        package="dynamixel_leadscrew_slider",
        executable="leadscrew_slider_controller",
        name="leadscrew_slider_controller",
        namespace="slider_top",
        condition=IfCondition(LaunchConfiguration("enable_top_slider")),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("slider_params_file"),
            {
                "motor.id": 2,
                "homing.min_soft_limit_range_mm": 240.0,
            },
        ],
        remappings=[
            ("/move_mm", "/slider_top/move_mm"),
            ("/current_position", "/slider_top/current_position"),
        ],
    )

    leadscrew_slider_controller_vertical_node = Node(
        package="dynamixel_leadscrew_slider",
        executable="leadscrew_slider_controller",
        name="leadscrew_slider_controller",
        namespace="slider_vertical",
        condition=IfCondition(LaunchConfiguration("enable_vertical_slider")),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("slider_params_file"),
            {
                "motor.id": 4,
                "homing.origin_offset_mm": 0.0,
                "homing.origin_reference": "max",
                "homing.return_to_position_mm": 0.0,
                "homing.seek_current_ma": 800,
                "homing.seek_current_ma_max": 1000,
                "homing.backoff_current_ma": 1000,
                "homing.min_soft_limit_range_mm": 350.0,
            },
        ],
        remappings=[
            ("/move_mm", "/slider_vertical/move_mm"),
            ("/current_position", "/slider_vertical/current_position"),
        ],
    )

    # If any leadscrew controller exits, shutdown the whole launch (fail-fast)
    shutdown_on_bottom_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=leadscrew_slider_controller_bottom_node,
            on_exit=[EmitEvent(event=Shutdown(reason="slider_bottom leadscrew exited"))],
        )
    )
    shutdown_on_top_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=leadscrew_slider_controller_top_node,
            on_exit=[EmitEvent(event=Shutdown(reason="slider_top leadscrew exited"))],
        )
    )
    shutdown_on_vertical_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=leadscrew_slider_controller_vertical_node,
            on_exit=[EmitEvent(event=Shutdown(reason="slider_vertical leadscrew exited"))],
        )
    )

    # ホーミング実行（起動後に順次呼び出し）
    home_bottom_after_delay = TimerAction(
        period=5.0,
        condition=IfCondition(LaunchConfiguration("enable_bottom_slider")),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/slider_bottom/home",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    home_top_after_delay = TimerAction(
        period=25.0,
        condition=IfCondition(LaunchConfiguration("enable_top_slider")),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/slider_top/home",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    home_vertical_after_delay = TimerAction(
        period=100.0,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("enable_vertical_slider"),
                    "' == 'true' and ('",
                    LaunchConfiguration("enable_bottom_slider"),
                    "' == 'true' or '",
                    LaunchConfiguration("enable_top_slider"),
                    "' == 'true')",
                ]
            )
        ),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/slider_vertical/home",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    home_vertical_immediate = TimerAction(
        period=0.1,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("enable_vertical_slider"),
                    "' == 'true' and '",
                    LaunchConfiguration("enable_bottom_slider"),
                    "' == 'false' and '",
                    LaunchConfiguration("enable_top_slider"),
                    "' == 'false'",
                ]
            )
        ),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/slider_vertical/home",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            slider_params_arg,
            enable_bottom_slider_arg,
            enable_top_slider_arg,
            enable_vertical_slider_arg,
            dynamixel_handler_node,
            ar_marker_web_client_node,
            calibrated_slider_controller_node,
            leadscrew_slider_controller_bottom_node,
            leadscrew_slider_controller_top_node,
            leadscrew_slider_controller_vertical_node,
            shutdown_on_bottom_exit,
            shutdown_on_top_exit,
            shutdown_on_vertical_exit,
            home_bottom_after_delay,
            home_top_after_delay,
            home_vertical_immediate,
            home_vertical_after_delay,
            force_sensor_node,
        ]
    )


