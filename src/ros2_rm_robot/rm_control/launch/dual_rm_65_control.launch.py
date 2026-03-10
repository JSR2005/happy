from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # arm1 - 使用arm_prefix参数而不是namespace
    control_node_1 = Node(
        package='rm_control',
        executable='rm_control',
        name='arm1_rm_control',  # 节点名
        parameters=[
            {'arm_prefix': 'arm1'},  # 关键：设置前缀参数
            {'follow': False},
            {'arm_type': 65}
        ],
        output='screen',
    )

    # arm2 - 使用arm_prefix参数而不是namespace
    control_node_2 = Node(
        package='rm_control',
        executable='rm_control',
        name='arm2_rm_control',  # 节点名
        parameters=[
            {'arm_prefix': 'arm2'},  # 关键：设置前缀参数
            {'follow': False},
            {'arm_type': 65}
        ],
        output='screen',
    )

    ld.add_action(control_node_1)
    ld.add_action(control_node_2)
    return ld