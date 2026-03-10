// force_control/src/admittance_controller.cpp
// 导纳控制器实现

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Dense>
#include <array>
#include <deque>

/**
 * 导纳控制器 (Admittance Controller)
 * 
 * 原理：根据感知的外力，自适应调整期望速度/位置
 * 
 * 数学模型:
 *   M * ẍ_d + D * ẋ_d + K * x_d = F_measured
 * 
 * 其中：
 *   M: 虚拟质量 (kg)
 *   D: 虚拟阻尼 (N·s/m)
 *   K: 虚拟刚度 (N/m)
 *   F_measured: 传感器测得的外力 (N)
 *   x_d: 期望位置 (m)
 * 
 * 应用场景：
 *   - 装配任务 (Assembly)
 *   - 表面接触任务 (Contact tasks)
 *   - 力-位协调操作
 */
class AdmittanceController : public rclcpp::Node {
public:
    AdmittanceController() : Node("admittance_controller") {
        RCLCPP_INFO(this->get_logger(), "初始化导纳控制器...");
        
        // ========== 声明参数 ==========
        // 虚拟动力学参数
        this->declare_parameter("virtual_mass", 5.0);           // kg
        this->declare_parameter("virtual_damping", 200.0);      // N·s/m
        this->declare_parameter("virtual_stiffness", 500.0);    // N/m
        
        // 控制参数
        this->declare_parameter("control_period_ms", 20);       // 20ms (50Hz)
        this->declare_parameter("force_deadband", 0.5);         // N
        this->declare_parameter("velocity_limit", 0.5);         // m/s
        this->declare_parameter("force_filter_alpha", 0.3);     // 低通滤波系数
        
        // 控制模式 (6 DOF: 3个平移 + 3个旋转)
        // 0: 位置控制, 1: 力控制
        this->declare_parameter("control_mode", 
                               std::vector<int64_t>{0, 0, 1, 0, 0, 0});
        
        // ========== 读取参数 ==========
        M_ = this->get_parameter("virtual_mass").as_double();
        D_ = this->get_parameter("virtual_damping").as_double();
        K_ = this->get_parameter("virtual_stiffness").as_double();
        
        control_period_ms_ = this->get_parameter("control_period_ms").as_int();
        dt_ = control_period_ms_ / 1000.0;
        
        force_deadband_ = this->get_parameter("force_deadband").as_double();
        v_max_ = this->get_parameter("velocity_limit").as_double();
        force_filter_alpha_ = this->get_parameter("force_filter_alpha").as_double();
        
        auto mode_param = this->get_parameter("control_mode").as_integer_array();
        for (int i = 0; i < 6 && i < mode_param.size(); ++i) {
            control_mode_[i] = static_cast<int>(mode_param[i]);
        }
        
        // 初始化状态
        force_.fill(0.0);
        velocity_.fill(0.0);
        acceleration_.fill(0.0);
        position_error_.fill(0.0);
        target_position_.fill(0.0);
        current_position_.fill(0.0);
        
        // ========== 创建订阅器 ==========
        // 力反馈订阅
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_feedback",
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                this->force_callback(msg);
            });
        
        // 目标位置订阅
        target_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose",
            rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->target_pos_callback(msg);
            });
        
        // 当前位置订阅
        current_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current_pose",
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->current_pos_callback(msg);
            });
        
        // ========== 创建发布器 ==========
        // 期望速度发布
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/desired_velocity", 
            rclcpp::SensorDataQoS());
        
        // 期望位置发布（用于验证）
        desired_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/desired_position", 
            rclcpp::QoS(10));
        
        // ========== 创建定时器 ==========
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_period_ms_),
            [this]() { this->control_loop(); });
        
        RCLCPP_INFO(this->get_logger(), 
                   "导纳控制器初始化完成 (M=%.1f, D=%.1f, K=%.1f, 周期=%dms)",
                   M_, D_, K_, control_period_ms_);
    }

private:
    /**
     * 力反馈回调 - 低通滤波处理
     */
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        // 提取力信息
        Eigen::Vector3d raw_force(
            msg->wrench.force.x,
            msg->wrench.force.y,
            msg->wrench.force.z
        );
        
        // 一阶低通滤波
        // 滤波后 = alpha * 新值 + (1-alpha) * 旧值
        for (int i = 0; i < 3; ++i) {
            force_[i] = force_filter_alpha_ * raw_force(i) + 
                       (1.0 - force_filter_alpha_) * force_[i];
        }
        
        // 死区处理（小于阈值的力视为0）
        for (int i = 0; i < 3; ++i) {
            if (std::abs(force_[i]) < force_deadband_) {
                force_[i] = 0.0;
            }
        }
        
        last_force_update_ = this->now();
    }
    
    /**
     * 目标位置回调
     */
    void target_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_position_[0] = msg->pose.position.x;
        target_position_[1] = msg->pose.position.y;
        target_position_[2] = msg->pose.position.z;
    }
    
    /**
     * 当前位置回调
     */
    void current_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_position_[0] = msg->pose.position.x;
        current_position_[1] = msg->pose.position.y;
        current_position_[2] = msg->pose.position.z;
    }
    
    /**
     * 主控制循环 (50Hz)
     */
    void control_loop() {
        // 检查数据是否超时
        auto now = this->now();
        if ((now - last_force_update_).seconds() > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                                *this->get_clock(), 
                                1000, 
                                "力反馈数据超时");
            return;
        }
        
        // 计算位置误差
        for (int i = 0; i < 3; ++i) {
            position_error_[i] = target_position_[i] - current_position_[i];
        }
        
        // ========== 导纳模型计算（每轴独立）==========
        // M * ẍ = F_measured - D * ẋ - K * x_error
        
        for (int i = 0; i < 3; ++i) {
            if (control_mode_[i] == 1) {  // 只有力控制模式下应用导纳
                // 加速度 (m/s²)
                acceleration_[i] = (force_[i] - D_ * velocity_[i] - K_ * position_error_[i]) / M_;
                
                // 积分得到速度 (m/s) - 梯形积分
                velocity_[i] += acceleration_[i] * dt_;
                
                // 速度限幅
                velocity_[i] = std::clamp(velocity_[i], -v_max_, v_max_);
                
                // 对应的位置变化 (用于验证)
                // position_[i] += velocity_[i] * dt_;
            } else {
                // 位置控制模式：直接输出位置
                velocity_[i] = 0.0;
                acceleration_[i] = 0.0;
            }
        }
        
        // ========== 发布期望速度给机械臂控制器 ==========
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = now;
        twist_msg.header.frame_id = "base_link";
        
        twist_msg.twist.linear.x = velocity_[0];
        twist_msg.twist.linear.y = velocity_[1];
        twist_msg.twist.linear.z = velocity_[2];
        
        // 旋转速度暂时设为0（可扩展到旋转控制）
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;
        
        velocity_pub_->publish(twist_msg);
        
        // ========== 周期性日志（可选）==========
        static int loop_count = 0;
        if (++loop_count % 50 == 0) {  // 每1秒输出一次
            RCLCPP_DEBUG(this->get_logger(),
                        "导纳控制状态 | "
                        "F=[%.2f, %.2f, %.2f] N | "
                        "v=[%.3f, %.3f, %.3f] m/s | "
                        "a=[%.3f, %.3f, %.3f] m/s²",
                        force_[0], force_[1], force_[2],
                        velocity_[0], velocity_[1], velocity_[2],
                        acceleration_[0], acceleration_[1], acceleration_[2]);
        }
    }
    
    // ========== 成员变量 ==========
    
    // 虚拟动力学参数
    double M_;  // 虚拟质量 (kg)
    double D_;  // 虚拟阻尼 (N·s/m)
    double K_;  // 虚拟刚度 (N/m)
    
    // 控制参数
    int control_period_ms_;
    double dt_;                 // 时间步长 (s)
    double force_deadband_;     // 力死区 (N)
    double v_max_;              // 最大速度 (m/s)
    double force_filter_alpha_; // 低通滤波系数
    
    std::array<int, 6> control_mode_;  // 每轴的控制模式
    
    // 状态变量（笛卡尔空间）
    std::array<double, 3> force_;            // 当前力 (N)
    std::array<double, 3> velocity_;         // 当前速度 (m/s)
    std::array<double, 3> acceleration_;     // 当前加速度 (m/s²)
    std::array<double, 3> position_error_;   // 位置误差 (m)
    std::array<double, 3> target_position_;  // 目标位置 (m)
    std::array<double, 3> current_position_; // 当前位置 (m)
    
    rclcpp::Time last_force_update_;
    
    // 订阅器
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pos_sub_;
    
    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pos_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdmittanceController>());
    rclcpp::shutdown();
    return 0;
}
