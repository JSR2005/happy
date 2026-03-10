#ifndef GRIPPER_MODULE_GRIPPER_CONTROLLER_HPP
#define GRIPPER_MODULE_GRIPPER_CONTROLLER_HPP

#include <memory>

#include "control_msgs/action/gripper_command.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace gripper_module {

/**
 * 夹爪控制器节点
 *
 * 功能：
 * - 通过官方 API 控制机械爪
 * - 支持位置控制和力控制
 * - 发布爪子工作坐标系和状态信息
 * - 提供 Action 接口用于抓取任务
 *
 * 接口：
 * - Action: /gripper_controller/gripper_cmd (GripperCommand)
 * - Topic: /gripper/state (关节状态)
 * - Topic: /gripper/force (力反馈)
 */
class GripperControllerNode : public rclcpp::Node {
 public:
  GripperControllerNode();
  ~GripperControllerNode() = default;

  using GripperCommandAction = control_msgs::action::GripperCommand;
  using GoalHandleGripperCommand =
      rclcpp_action::ServerGoalHandle<GripperCommandAction>;

 private:
  // 参数
  double max_gripper_effort_;     // 最大夹爪力 (N)
  double gripper_speed_;          // 夹爪速度
  std::string gripper_frame_id_;  // 夹爪工作坐标系

  // 状态
  double current_position_ = 0.0;  // 当前位置 (0-1, 0=打开, 1=闭合)
  double current_effort_ = 0.0;    // 当前力
  bool object_detected_ = false;   // 是否检测到物体

  // 订阅器和发布器
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_state_pub_;

  // Action
  rclcpp_action::Server<GripperCommandAction>::SharedPtr gripper_action_server_;

  // 回调函数
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // Action 处理
  rclcpp_action::GoalResponse handle_goal(
      const std::shared_ptr<const GripperCommandAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

  void execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

  // 夹爪控制
  void gripper_command(double position, double effort);
  void call_official_api(double target_pos, double force);
};

}  // namespace gripper_module

#endif
