# ros2_rm_robot/rm_driver/launch/rm_65_driver.launch.py（改造后）
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 声明参数：命名空间、配置文件名称
    ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for arm (e.g., arm1_, arm2_)"
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value="rm_65_config.yaml",
        description="Driver config file name (without path)"
    )

    # 2. 加载配置文件
    config_path = PathJoinSubstitution([
        get_package_share_directory('rm_driver'),
        'config',
        LaunchConfiguration("config_file")
    ])

    # 3. 启动driver节点（带namespace）
    rm_driver_node = Node(
        package="rm_driver",
        executable="rm_driver",
        namespace=LaunchConfiguration("namespace"),  # 关键：命名空间隔离
        parameters=[config_path],
        output='screen'
    )

    return LaunchDescription([
        ns_arg,
        config_arg,
        rm_driver_node
    ])