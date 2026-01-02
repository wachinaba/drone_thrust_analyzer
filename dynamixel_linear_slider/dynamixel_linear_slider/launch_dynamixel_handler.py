from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

pkg_name = 'dynamixel_handler'

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'config_dynamixel_handler.yaml'
    )

    node = Node(
        package=pkg_name, # パッケージ名
        executable='dynamixel_handler', # 実行ファイル名
        name='dynamixel_handler', # node名
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )
    # emulate_ttyについては以下参照 https://answers.ros.org/question/332829/no-stdout-logging-output-in-ros2-using-launch/

    ld.add_action(node)
    # 複数ノードを追加する場合は，configN,nodeNを作ってld.add_action(nodeN)?

    return ld
