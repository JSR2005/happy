import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 双臂驱动节点
    rm_dual_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_driver'),'launch', 'dual_rm_65_driver.launch.py'))
    )

    # 双臂控制节点
    rm_dual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_control'),'launch', 'dual_rm_65_control.launch.py'))
    )

    # 双臂MoveIt配置
    rm_dual_moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_65_dual_config'),'launch', 'dual_real_moveit_demo_6f.launch.py'))
    )
    rm_dual_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_description'),'launch', 'display.launch.py'))
    )

    return LaunchDescription([
        rm_dual_driver,
        rm_dual_control,
        rm_dual_moveit_config,
        rm_dual_display 

    ])