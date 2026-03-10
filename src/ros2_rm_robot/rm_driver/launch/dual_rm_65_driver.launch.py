import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # arm1
    arm1_config = os.path.join(get_package_share_directory('rm_driver'),'config','arm1_rm_65_config.yaml')
    with open(arm1_config,'r') as f:
        params1 = yaml.safe_load(f)["rm_driver"]["ros__parameters"]

    # arm2
    arm2_config = os.path.join(get_package_share_directory('rm_driver'),'config','arm2_rm_65_config.yaml')
    with open(arm2_config,'r') as f:
        params2 = yaml.safe_load(f)["rm_driver"]["ros__parameters"]

    return LaunchDescription([
        Node(
            package="rm_driver",
            executable="rm_driver",
            namespace="arm1",
            parameters=[params1],
            output='screen'
        ),
        Node(
            package="rm_driver",
            executable="rm_driver",
            namespace="arm2",
            parameters=[params2],
            output='screen'
        )
    ])