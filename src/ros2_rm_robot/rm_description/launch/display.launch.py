import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
         # 声明参数 link6_type
    # declare_link6_type_arg = DeclareLaunchArgument(
    #     'link6_type',
    #     default_value='Link6_6fb',
    #     description='Type of link6'
    # )
    # 获取包路径
    rm_description_dir = get_package_share_directory('rm_description')
    
    # 声明参数
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(rm_description_dir, 'urdf', 'dual_rm_65.urdf.xacro'),
        description='Path to the robot URDF/XACRO file'
    )
    
    # 处理XACRO文件，生成URDF内容
    robot_description = Command(
        [
            'xacro ',
            LaunchConfiguration('model')
            # 'link6_type:=', LaunchConfiguration('link6_type')
        ]
    )

    # 配置参数服务器：发布机器人描述
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 关节状态聚合器：合并来自两个手臂驱动节点的 /arm1/joint_states 和 /arm2/joint_states
    # 并将合并后的话题发布到 /joint_states，供 robot_state_publisher 使用
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_aggregator', # 更改节点名称以明确其作用
        parameters=[{'source_list': ['arm1/joint_states', 'arm2/joint_states']}]
    )
    
    # RViz2节点
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(rm_description_dir, 'rviz.rm_65.rviz')],
    #     output='screen'
    # )

    # 组装launch描述
    return LaunchDescription([
        # declare_link6_type_arg,
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        # rviz_node
    ])