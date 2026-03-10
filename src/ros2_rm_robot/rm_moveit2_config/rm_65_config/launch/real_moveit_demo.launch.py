################################################################################
# MoveIt2 运动规划框架启动文件
# 
# 【功能概述】
# 本文件是MoveIt2的启动配置脚本，用于启动实体机械臂(Real Robot)的运动规划环境
# 
# 【核心组件】
# 1. Move Group（运动规划核心）- 负责轨迹规划、碰撞检测、逆运动学等
# 2. RViz（可视化工具）- 用于实时显示机械臂状态和规划结果
# 
# 【与rm_driver的关系】
# - rm_driver：负责ROS2与硬件通信（TCP/UDP）
# - MoveIt：基于rm_driver的话题数据进行运动规划
# - 数据流向：rm_driver发布关节状态 → MoveIt规划 → rm_driver执行命令
################################################################################

# ============ MoveIt2配置工具导入 ============
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

# ============ ROS2 Launch框架导入 ============
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,      # 声明启动参数
    IncludeLaunchDescription,   # 包含其他launch文件
)
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,        # 添加可调试节点
    DeclareBooleanLaunchArg,    # 声明布尔参数
)
from launch.substitutions import LaunchConfiguration  # 获取启动参数值
from launch_ros.parameter_descriptions import ParameterValue  # 参数值封装


def generate_launch_description():
    """
    【主启动函数】
    功能：生成完整的MoveIt2启动配置
    返回值：LaunchDescription对象（包含所有启动节点和配置）
    """
    
    # ============ 第一步：加载MoveIt2配置 ============
    # 加载rm_65的MoveIt2配置文件（包括URDF、运动学参数、规划参数等）
    # 参数说明：
    #   - "rm_65_description": 机械臂的URDF模型ID
    #   - "package_name": 配置文件所在功能包名
    moveit_config = MoveItConfigsBuilder(
        "rm_65_description", 
        package_name="rm_65_config"
    ).to_moveit_configs() 

    # ============ 第二步：创建启动描述对象 ============
    # 所有节点和参数都添加到这个对象中
    ld = LaunchDescription()

    # ============ 第三步：启动MoveIt核心组件 ============
    # 启动move_group节点（运动规划引擎）
    my_generate_move_group_launch(ld, moveit_config)
    
    # 启动RViz可视化工具
    my_generate_moveit_rviz_launch(ld, moveit_config)

    # generate_rsp_launch(ld, moveit_config)  # 可选：状态发布器（已注释）

    return ld


################################################################################
# Move Group 启动配置函数
# 【功能】启动MoveIt2的核心运动规划模块
################################################################################
def my_generate_move_group_launch(ld, moveit_config):
    """
    【功能函数】
    功能：配置并启动Move Group节点
    参数：
        - ld: LaunchDescription对象（用于添加节点）
        - moveit_config: MoveIt配置对象（包含机械臂配置信息）
    
    【Move Group的作用】
    - 路径规划：使用RRT、PRM等算法规划无碰撞路径
    - 逆运动学计算：根据目标位姿计算关节角
    - 碰撞检测：检测规划路径是否与环境碰撞
    - 轨迹执行管理：监控轨迹执行状态
    """

    # ============ 声明启动参数 ============
    # 调试模式标志（false=不调试，true=进入GDB调试）
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    
    # 允许轨迹执行（false=仅规划不执行，true=执行规划的轨迹）
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    
    # 发布监控的规划场景（用于RViz显示）
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    
    # 加载自定义的MoveGroup能力（高级功能扩展）
    # 注意：默认为空字符串表示使用默认能力集
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    
    # 禁用指定的MoveGroup能力（用于关闭某些高级功能）
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # 监控动力学信息（是否从/joint_states中复制关节动力学数据）
    # 注意：大多数MoveGroup功能不需要这个，设为False可减少计算量
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    # ============ 获取启动参数值 ============
    # 获取"publish_monitored_planning_scene"参数值，供后续使用
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # ============ Move Group核心配置参数 ============
    move_group_configuration = {
        # 发布机械臂的语义信息（关节名称、碰撞对象等）
        "publish_robot_description_semantic": True,
        
        # 是否允许执行规划的轨迹
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        
        # 高级能力扩展（如轨迹执行监控、碰撞检测等）
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        
        # 禁用的能力
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        
        # 发布规划场景给RViz（显示机械臂周围的环境）
        "publish_planning_scene": should_publish,
        
        # 发布几何更新（环境变化时更新RViz显示）
        "publish_geometry_updates": should_publish,
        
        # 发布状态更新（关节角度变化时更新）
        "publish_state_updates": should_publish,
        
        # 发布变换更新（TF坐标系变换）
        "publish_transforms_updates": should_publish,
        
        # 监控机械臂动力学（用于力控应用）
        "monitor_dynamics": False,
    }

    # ============ 轨迹执行配置 ============
    trajectory_execution = {
        # MoveIt是否管理控制器（false=不管理，rm_control管理）
        "moveit_manage_controllers": False,
        
        # 允许的执行时间缩放（1.2 = 允许延长20%的执行时间，用于容错）
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        
        # 允许的目标到达时间余量（秒）
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        
        # 允许的起始位置容差（度）
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }

    # ============ 合并所有MoveGroup参数 ============
    # 参数优先级：后面的参数覆盖前面的
    move_group_params = [
        moveit_config.to_dict(),              # 基础配置（URDF、规划管道等）
        move_group_configuration,              # Move Group特定配置
        trajectory_execution,                  # 轨迹执行配置
    ]

    # ============ 启动Move Group节点 ============
    add_debuggable_node(
        ld,
        # 节点信息
        package="moveit_ros_move_group",       # 官方MoveIt包
        executable="move_group",               # 可执行文件名
        
        # 调试相关
        commands_file=str(
            moveit_config.package_path / "launch" / "gdb_settings.gdb"
        ),
        output="screen",                       # 输出到屏幕（便于查看日志）
        parameters=move_group_params,          # 传入所有参数
        extra_debug_args=["--debug"],          # 调试模式额外参数
        
        # 环境变量（某些OpenGL代码需要）
        additional_env={"DISPLAY": ":0"},
    )
    return ld



################################################################################
# RViz可视化启动配置函数
# 【功能】启动RViz用于可视化机械臂和规划结果
################################################################################
def my_generate_moveit_rviz_launch(ld, moveit_config):
    """
    【功能函数】
    功能：启动RViz2节点用于可视化MoveIt规划
    参数：
        - ld: LaunchDescription对象
        - moveit_config: MoveIt配置对象
    
    【RViz的作用】
    - 实时显示机械臂当前位置和关节状态
    - 显示规划的运动轨迹
    - 显示碰撞检测结果
    - 提供交互式操作界面（IK目标设置、拖拽操作等）
    """
    
    # ============ 声明RViz启动参数 ============
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    
    # 声明RViz配置文件路径参数
    # 默认值指向MoveIt生成的RViz配置文件（包含所有插件、视图设置等）
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(
                moveit_config.package_path / "config/moveit.rviz"
            ),
        )
    )

    # ============ RViz所需的参数 ============
    # 这些参数让RViz知道规划管道和运动学求解器的信息
    rviz_parameters = [
        # 规划管道配置（RRT、OMPL等规划器信息）
        moveit_config.planning_pipelines,
        
        # 运动学求解器配置（IKFast、KDL等）
        moveit_config.robot_description_kinematics,
    ]

    # ============ 启动RViz2节点 ============
    add_debuggable_node(
        ld,
        # 节点信息
        package="rviz2",                      # ROS2官方可视化工具
        executable="rviz2",                   # 可执行文件
        output="log",                         # 输出到日志（减少屏幕输出）
        respawn=False,                        # 节点异常时不自动重启
        
        # RViz启动参数
        arguments=[
            "-d",                             # -d: 指定配置文件
            LaunchConfiguration("rviz_config") # 配置文件路径
        ],
        parameters=rviz_parameters,           # 规划和运动学参数
    )

    return ld

