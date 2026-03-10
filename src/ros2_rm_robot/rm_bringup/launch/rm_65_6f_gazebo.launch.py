import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():

    rm_65_gazebo_up = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_gazebo')),'launch', 'gazebo_65_6f_demo.launch.py'))
    )
    rm_65_gazebo_moveit = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_65_6f_gripper_config')),'launch', 'gazebo_moveit_demo_6f.launch.py'))
    )
        # 节点：用于在Rviz中生成障碍物
    spawn_obstacle_node = Node(
       package='rm_gazebo',
       executable='spawn_obstacle.py',  
       name='spawn_obstacle',
       output='screen'   
     )
        # 障碍物SDF模型文件的路径
    obstacle_sdf_path = os.path.join(
        get_package_share_directory('rm_gazebo'),
        'models', 'box_obstacle', 'model.sdf')

    # 节点：在Gazebo中生成物理障碍物
    spawn_obstacle_gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'box_obstacle', '-file', obstacle_sdf_path],
        output='screen'
    )

    return LaunchDescription([
        rm_65_gazebo_up,
        rm_65_gazebo_moveit,
        # 延迟几秒启动，等待MoveIt和Gazebo完全加载
        TimerAction(
            period=6.0,
            actions=[spawn_obstacle_node]),
        TimerAction(
            period=5.0,
            actions=[spawn_obstacle_gazebo])
    ])