#include "gripper_module/gripper_controller.hpp"

#include <memory>

namespace gripper_module {

GripperControllerNode::GripperControllerNode() : Node("gripper_controller") {
  RCLCPP_INFO(this->get_logger(), "初始化夹爪控制器...");

  // 声明参数
  this->declare_parameter("max_gripper_effort", 100.0);
  this->declare_parameter("gripper_speed", 50.0);
  this->declare_parameter("gripper_frame_id", "gripper_base");

  // 读取参数
  max_gripper_effort_ = this->get_parameter("max_gripper_effort").as_double();
  gripper_speed_ = this->get_parameter("gripper_speed").as_double();
  gripper_frame_id_ = this->get_parameter("gripper_frame_id").as_string();

  // 创建订阅器
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/gripper/joint_state", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->joint_state_callback(msg);
      });

  force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/gripper/force", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        this->force_callback(msg);
      });

  // 创建发布器
  gripper_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/gripper/state", 10);

  // 创建 Action Server
  gripper_action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      this, "/gripper_controller/gripper_cmd",
      std::bind(&GripperControllerNode::handle_goal, this,
                std::placeholders::_1),
      std::bind(&GripperControllerNode::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&GripperControllerNode::execute, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "✓ 夹爪控制器初始化完成 (最大力: %.1f N, 速度: %.1f mm/s)",
              max_gripper_effort_, gripper_speed_);
}

void GripperControllerNode::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // 更新夹爪状态
  if (!msg->position.empty()) {
    current_position_ = msg->position[0];  // 0-1: 0=打开, 1=闭合
  }
  if (!msg->effort.empty()) {
    current_effort_ = msg->effort[0];
  }
}

void GripperControllerNode::force_callback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  // 检测是否有物体被夹住
  double force_magnitude = std::sqrt(msg->wrench.force.x * msg->wrench.force.x +
                                     msg->wrench.force.y * msg->wrench.force.y +
                                     msg->wrench.force.z * msg->wrench.force.z);

  object_detected_ = (force_magnitude > 5.0);  // > 5N 则检测到物体

  if (object_detected_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "✓ 检测到物体 (力: %.2f N)", force_magnitude);
  }
}

rclcpp_action::GoalResponse GripperControllerNode::handle_goal(
    const std::shared_ptr<const GripperCommandAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "收到夹爪命令: 位置=%.2f, 力=%.2f",
              goal->command.position, goal->command.max_effort);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperControllerNode::handle_cancel(
    const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "取消夹爪命令");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperControllerNode::execute(
    const std::shared_ptr<GoalHandleGripperCommand> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GripperCommandAction::Feedback>();
  auto result = std::make_shared<GripperCommandAction::Result>();

  RCLCPP_INFO(this->get_logger(), "执行夹爪命令...");

  // 发送命令到硬件
  gripper_command(goal->command.position, goal->command.max_effort);

  // 模拟执行过程
  rclcpp::Rate loop_rate(10);     // 10 Hz
  for (int i = 0; i < 50; ++i) {  // 5 秒
    if (goal_handle->is_canceling()) {
      result->reached_goal = false;
      goal_handle->canceled(result);
      return;
    }

    feedback->position = current_position_;
    feedback->effort = current_effort_;
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  result->reached_goal = true;
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "✓ 夹爪命令执行完成");
}

void GripperControllerNode::gripper_command(double position, double effort) {
  // 调用官方 API 或底层驱动
  // 这里是伪代码示例

  RCLCPP_INFO(this->get_logger(), "发送夹爪命令到硬件 (位置: %.2f, 力: %.2f)",
              position, effort);

  // 实际实现需要调用官方 API
  // call_official_api(position, effort);

  // 发布当前状态
  sensor_msgs::msg::JointState state_msg;
  state_msg.header.stamp = this->now();
  state_msg.name = {"gripper"};
  state_msg.position = {position};
  state_msg.effort = {effort};
  gripper_state_pub_->publish(state_msg);
}

void GripperControllerNode::call_official_api(double target_pos, double force) {
  // 这里集成官方 RM API
  // 伪代码示例：
  //
  // try {
  //     rm_api->set_gripper_position(target_pos);
  //     rm_api->set_gripper_force(force);
  //     rm_api->move_gripper();
  // } catch (const std::exception& e) {
  //     RCLCPP_ERROR(this->get_logger(), "夹爪命令执行失败: %s", e.what());
  // }
}

}  // namespace gripper_module

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gripper_module::GripperControllerNode)
