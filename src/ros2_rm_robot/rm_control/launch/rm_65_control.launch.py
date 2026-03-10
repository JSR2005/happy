from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数（可选：外部传入臂型/跟随模式）
    left_arm_type = DeclareLaunchArgument(
        "left_arm_type",
        default_value="65",
        description="Left arm type: 65/75/eco65..."
    )
    right_arm_type = DeclareLaunchArgument(
        "right_arm_type",
        default_value="65",
        description="Right arm type: 65/75/eco65..."
    )

    # 左手臂 rm_control 节点
    left_rm_control = Node(
        package="rm_control",
        executable="rm_control_node",  # 确保可执行文件名称正确
        name="left_rm_control",
        parameters=[
            {"arm_prefix": "arm1_"},       # 前缀区分
            {"arm_type": LaunchConfiguration("left_arm_type")},
            {"follow": False}              # 低跟随模式（可根据需求改true）
        ],
        output="screen"
    )

    # 右手臂 rm_control 节点
    right_rm_control = Node(
        package="rm_control",
        executable="rm_control_node",
        name="right_rm_control",
        parameters=[
            {"arm_prefix": "arm2_"},      # 前缀区分
            {"arm_type": LaunchConfiguration("right_arm_type")},
            {"follow": False}
        ],
        output="screen"
    )

    return LaunchDescription([
        left_arm_type,
        right_arm_type,
        left_rm_control,
        right_rm_control
    ])