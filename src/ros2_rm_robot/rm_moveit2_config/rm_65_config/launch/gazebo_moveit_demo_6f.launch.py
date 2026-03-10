from moveit_configs_utils import MoveItConfigsBuilder
#MoveIt2 官方提供的 “配置构建工具”，简化手动编写大量 MoveIt 配置的工作，可以一键加载机械臂的 URDF/XACRO 模型、运动学配置、规划器参数等；
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("rm_65_description", package_name="rm_65_config").to_moveit_configs() 
    
    moveit_config = (
        MoveItConfigsBuilder("rm_65_description", package_name="rm_65_config")
        .to_moveit_configs()
    )

    # Get the robot description xacro file
    robot_description_xacro_file = PathJoinSubstitution(
        [FindPackageShare("rm_65_config"), "config", "rm_65_6fb_description.urdf.xacro"]
    )

    # Generate the robot description XML
    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_description_xacro_file,
                " ",
                "link6_type:=Link6_6fb",
            ]
        ),
        value_type=str,
    )

    # Overwrite the robot_description in moveit_config
    moveit_config.robot_description = {"robot_description": robot_description_content}
    # 创建启动描述实例（所有节点/参数都要加进去）
    ld = LaunchDescription()

    # 启动move_group
    my_generate_move_group_launch(ld, moveit_config)
    # 启动rviz
    my_generate_moveit_rviz_launch(ld, moveit_config, robot_description_content)

    return ld


def my_generate_move_group_launch(ld, moveit_config):

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True) # 是否允许执行轨迹
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True) # 是否发布规划场景
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
    # 拼接最终的参数列表：MoveIt配置 + move_group配置 + 仿真时间
    # 创建一个包含仿真时间的基础参数字典
    common_params = {"use_sim_time": True}

    # 将所有参数字典合并
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        common_params,
    ]

    add_debuggable_node(# 添加move_group节点到启动清单
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],# 调试模式参数
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": ":0"},
    )
    return ld

def my_generate_moveit_rviz_launch(ld, moveit_config, robot_description_content):
    """Launch file for rviz"""

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),# 默认RViz配置文件路径
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines, # 规划器配置（比如OMPL规划器）
        moveit_config.robot_description_kinematics,# 机械臂运动学配置
        {"robot_description": robot_description_content}, # 关键：为RViz补上robot_description
    ]
    rviz_parameters.append({"use_sim_time": True})

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )
    return ld