from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_params(context, *args, **kwargs):
    # 1. 处理 use_sim_time 类型转换
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_str.lower() == "true"

    # 2. 定义路径
    moveit_config_package = "rm_65_dual_config"
    moveit_config_dir = get_package_share_directory(moveit_config_package)
    rm_description_dir = get_package_share_directory("rm_description")

    # 3. 读取核心配置文件内容
    # SRDF
    srdf_path = os.path.join(moveit_config_dir, "config", "dual_rm_65.srdf")
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    
    # 运动学配置
    kinematics_path = os.path.join(moveit_config_dir, "config", "kinematics.yaml")
    with open(kinematics_path, 'r') as f:
        kinematics_params = yaml.safe_load(f)
    
    # 控制器配置
    controller_path = os.path.join(moveit_config_dir, "config", "moveit_controller_manager.yaml")
    with open(controller_path, 'r') as f:
        controller_params = yaml.safe_load(f)
    
    # 关节限位配置
    joint_limits_path = os.path.join(moveit_config_dir, "config", "joint_limits.yaml")
    with open(joint_limits_path, 'r') as f:
        joint_limits_params = yaml.safe_load(f)
    
    # OMPL 规划配置（已包含碰撞检测参数）
    ompl_path = os.path.join(moveit_config_dir, "config", "ompl_planning.yaml")
    with open(ompl_path, 'r') as f:
        ompl_params = yaml.safe_load(f)

    # 4. 生成 URDF 内容
    urdf_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([rm_description_dir, "urdf", "dual_rm_65.urdf.xacro"]),
            " use_sim_time:=", use_sim_time_str
        ]),
        value_type=str
    )

    # 5. 通用参数（仅保留合法参数，删除错误的碰撞参数）
    common_params = {
        "robot_description": urdf_content,
        "robot_description_semantic": srdf_content,
        "robot_description_kinematics": kinematics_params,
        "robot_description_planning": joint_limits_params,
        "ompl_planning": ompl_params,
        "use_sim_time": use_sim_time,
        **controller_params
    }

    # MoveGroup 节点（核心：传递所有参数）
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            common_params,
            {
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "trajectory_execution": {
                    "allowed_execution_duration_scaling": 1.2,
                    "allowed_goal_duration_margin": 0.5,
                    "allowed_start_tolerance": 0.01,
                    "moveit_manage_controllers": True
                },
                # 仅添加MoveGroup合法的碰撞参数
                "start_state_max_bounds_error": 0.1,
                "planning_scene_monitor_options": {
                    "name": "planning_scene_monitor",
                    "robot_description": "robot_description",
                    "joint_state_topic": "/joint_states",
                    "attached_collision_object_topic": "/move_group/planning_scene_monitor",
                    "publish_planning_scene_topic": "/move_group/publish_planning_scene",
                    "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
                    "wait_for_initial_state_timeout": 10.0
                }
            }
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # RViz 节点
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([moveit_config_dir, "config", "moveit.rviz"])],
        parameters=[common_params]
    )

    return [move_group, rviz]

def generate_launch_description():
    # 声明参数
    declare_arm_type = DeclareLaunchArgument(
        "arm_type",
        default_value="65",
        description="RM机械臂型号"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="是否使用仿真时间"
    )

    # 加载动态参数
    load_params_func = OpaqueFunction(function=load_params)

    return LaunchDescription([
        declare_arm_type,
        declare_use_sim_time,
        load_params_func
    ])