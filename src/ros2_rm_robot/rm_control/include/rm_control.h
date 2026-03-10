#ifndef RM_CONTROL_H
#define RM_CONTROL_H

/*################################################################################
# rm_control模块 - 轨迹执行控制器
# 
# 【核心职责】
# 1. 接收MoveIt规划的轨迹（含多个路点）
# 2. 使用三次样条插值进行轨迹光滑化
# 3. 以20ms周期发送插值后的关节角度到rm_driver
# 4. 等待轨迹执行完成
#
# 【工作流程】
# MoveIt规划 → Action请求 → 三次样条插值 → 周期发送 → rm_driver执行 → 硬件运动
#
# 【关键算法】
# - 三次样条插值（Cubic Spline Interpolation）
#   作用：将MoveIt的稀疏路点转换为平滑连续的轨迹
#   优点：避免关节突跃，保证运动平滑
################################################################################*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <iostream>
#include "control_msgs/action/follow_joint_trajectory.hpp"  // ROS标准轨迹跟踪Action
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

//RM Robot msg
#include "rm_ros_interfaces/msg/jointpos.hpp"  // 关节位置消息（透传给rm_driver）
//#include "rm_ros_interfaces/msg/jointpos75.hpp"
// 【新增】引入机械爪控制的msg（需确认rm_ros_interfaces中机械爪msg名称，如Gripperset/Gripperpick）
#include "rm_ros_interfaces/msg/gripperset.hpp"
#include "rm_ros_interfaces/msg/gripperpick.hpp"
#include "control_msgs/action/gripper_command.hpp"

/* 使用变长数组 */
#include <vector>
#include <algorithm>
#include <thread>  // 添加这行 - 修复thread错误
#include <functional>  // 添加这行 - 修复placeholders错误

using namespace std;

/*################################################################################
# Rm_Control类 - 轨迹执行核心
################################################################################*/
class Rm_Control : public rclcpp::Node
{
public:
    // ============ Action类型定义 ============
    // FollowJointTrajectory: ROS标准的轨迹跟踪Action
    // 接口格式：
    //   - Goal: 传入轨迹（含多个路点、时间戳等）
    //   - Result: 返回执行结果（error_code、error_string）
    //   - Feedback: 实时反馈（当前执行进度）
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    // ============ 构造和析构函数 ============
    explicit Rm_Control(std::string name);
    ~Rm_Control(){}

    // ============ 定时回调函数 ============
    // 功能：每20ms执行一次，发送插值后的关节角度
    void timer_callback();

private:
    // ============ 消息对象 ============
    // 用于存储要发送给rm_driver的关节位置
    rm_ros_interfaces::msg::Jointpos joint_msg;
    // rm_ros_interfaces::msg::Jointpos75 joint7_msg;
       // 新增：机械臂前缀（区分左/右臂，如 "left_"、"right_"）
    std::string arm_prefix_ = "";
    
    // ============ 机械臂配置参数 ============
    int arm_type_ = 75;          // 机械臂型号（75=7轴，65=6轴）
    bool follow_ = false;        // 跟随模式标志

       // 【新增】机械爪Action服务器（使用arm_prefix区分）
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperCommand>;
    rclcpp_action::Server<GripperCommand>::SharedPtr gripper_action_server;

    // 【新增】爪控指令发布器（对应rm_driver的话题）
    rclcpp::Publisher<rm_ros_interfaces::msg::Gripperpick>::SharedPtr gripper_pick_pub;
    rclcpp::Publisher<rm_ros_interfaces::msg::Gripperset>::SharedPtr gripper_pos_pub;


    // 【新增】爪控结果订阅器（接收rm_driver的执行结果）

  

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_pos_result_sub;

    // 【新增】爪控结果缓存（用于Action反馈）
    bool gripper_result = false;
   
       // 【新增】机械爪回调（统一的处理函数）
    rclcpp_action::GoalResponse handle_gripper_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperCommand::Goal> goal);
    rclcpp_action::CancelResponse handle_gripper_cancel(
        const std::shared_ptr<GoalHandleGripper> goal_handle);
    void handle_gripper_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);
    void execute_gripper_command(const std::shared_ptr<GoalHandleGripper> goal_handle);

    // 【新增】结果回调声明
    void gripper_pick_result_cb(const std_msgs::msg::Bool::SharedPtr msg);
    void gripper_pos_result_cb(const std_msgs::msg::Bool::SharedPtr msg);

    
    // ============ Action服务器 ============
    // 用于接收MoveIt的轨迹规划结果
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // ============ ROS2话题发布器 ============
    // 发布关节位置指令到rm_driver（话题：/rm_driver/movej_canfd_cmd）
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher;
    // rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos75>::SharedPtr joint_pos_publisher_75;

    // ============ ROS2话题订阅器 ============
    // 订阅停止命令（话题：rm_driver/move_stop_cmd）
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Get_Move_Stop_Cmd;

    // ============ 定时器 ============
    // 20ms周期定时器，用于定周期发送关节指令
    rclcpp::TimerBase::SharedPtr State_Timer;

    // ============ Action处理函数 ============
    // handle_goal: 处理MoveIt发来的轨迹请求
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const FollowJointTrajectory::Goal> goal
    );
    
    // handle_cancel: 处理取消请求
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle
    );
    
    // execute_move: 执行轨迹（三次样条插值 + 周期发送）
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    
    // handle_accepted: 处理接受的Goal
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    
    // get_move_stop_callback: 处理停止命令
    void get_move_stop_callback(std_msgs::msg::Empty::SharedPtr msg);
};

#endif // Rm_Control_H