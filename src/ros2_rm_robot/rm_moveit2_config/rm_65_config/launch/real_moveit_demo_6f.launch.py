# 导入MoveIt配置工具
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

# 导入ROS 2启动相关模块
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 构建MoveIt配置，指定机械臂模型包为rm_65_config
    # 使用6自由度夹爪配置（Link6_6f）
    moveit_config = (
        MoveItConfigsBuilder("rm_65_description", package_name="rm_65_config")
        .robot_description(file_path="config/rm_65_6fb_description.urdf.xacro", mappings={"link6_type": "Link6_6f"})
        # 注释掉的配置项：
        # .robot_description_semantic(file_path="config/rm_65_description.srdf")
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    # 创建启动描述对象
    ld = LaunchDescription()

    # 启动move_group节点（运动规划组）
    my_generate_move_group_launch(ld, moveit_config)
    # 启动RVIZ可视化界面
    my_generate_moveit_rviz_launch(ld, moveit_config)

    # 注释掉的机器人状态发布器启动
    # generate_rsp_launch(ld, moveit_config)

    return ld


def my_generate_move_group_launch(ld, moveit_config):
    # 添加调试参数
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    # 允许轨迹执行
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    # 发布监控的规划场景
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # 加载非默认的MoveGroup功能
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # 禁用某些默认的MoveGroup功能
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # 不复制动力学信息从/joint_states到内部机器人监控
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # MoveGroup配置参数
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # 发布物理机器人的规划场景，使RVIZ插件能知道实际机器人状态
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    # 轨迹执行参数
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        trajectory_execution,
    ]

    # 添加可调试的move_group节点
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # 设置显示变量，以防内部使用OpenGL代码
        additional_env={"DISPLAY": ":0"},
    )
    return ld

def my_generate_moveit_rviz_launch(ld, moveit_config):
    """RVIZ启动文件"""
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    # 添加RVIZ2节点
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
