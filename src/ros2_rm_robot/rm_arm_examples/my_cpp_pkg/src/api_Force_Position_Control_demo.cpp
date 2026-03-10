//
// Created by ubuntu on 24-7-11.
// 力位混合控制演示程序
// 功能：通过ROS2发布力位混合控制命令到机械臂驱动
//

// ========================================= 程序概述 =========================================
// 
// 【程序名称】力位混合控制（Force-Position Hybrid Control）演示程序
// 
// 【核心功能】
// 本程序实现了一个ROS2节点，用于控制机械臂进行力位混合运动。在接触任务中，既需要控制
// 接触力（力控），又需要控制运动轨迹（位控）。通过将两者结合，可以实现更灵活的操作。
//
// 【控制策略说明】
// - 水平方向（X/Y轴）：采用浮动模式，允许自由运动，不施加力控约束
// - 竖直方向（Z轴）：采用力跟踪模式，保持20N的恒定压力（接近表面）
// - 旋转方向（Mx/My/Mz）：全部锁定，保持工具姿态不变
//
// 【工作流程】
// 1. 初始化ROS2节点，创建发布器连接到机械臂驱动
// 2. 启动50Hz定时器（每20ms执行一次）
// 3. 每次定时器触发时，构建力位混合参数消息
// 4. 将参数消息发布到/rm_driver/set_force_postion_new_cmd话题
// 5. 机械臂驱动收到指令并执行力位混合控制
//
// 【典型应用场景】
// - 表面接触操作：如打磨、抛光、去毛刺等需要恒定压力的作业
// - 装配任务：需要控制接触力以避免产品受损
// - 触觉反馈控制：实时调整接触力以完成精细操作
//

// ============ 标准库头文件 ============
#include <iostream>     // 标准输入输出
#include <chrono>       // 时间库
#include <functional>   // 函数包装器
#include <memory>       // 智能指针
#include <unistd.h>     // POSIX API
#include <thread>       // 多线程
#include <cmath>        // 数学函数

// ============ ROS2相关头文件 ============
#include "rclcpp/rclcpp.hpp"                                  // ROS2核心库
#include "std_msgs/msg/bool.hpp"                             // 标准布尔消息
#include "std_msgs/msg/empty.hpp"                            // 标准空消息
#include "rm_ros_interfaces/msg/movejp.hpp"                  // 关节空间点对点运动消息
#include "rm_ros_interfaces/msg/setforceposition.hpp"        // 设置力位混合参数消息
#include "rm_ros_interfaces/msg/setforcepositionnew.hpp"     // 新版力位混合参数消息
#include "rm_ros_interfaces/msg/forcepositionmove.hpp"       // 力位混合运动消息

// ============ 机械臂驱动头文件 ============
#include "rm_driver/rm_define.h"                             // 机械臂定义
#include "rm_driver/rm_interface.h"                          // 机械臂接口
#include "rm_driver/rm_interface_global.h"                   // 全局接口
#include "rm_driver/rm_service.h"                            // 服务相关
#include "rm_driver/rm_driver.h"                             // 驱动相关




// 使用std::chrono_literals命名空间，支持20ms这样的时间字面量
using namespace std::chrono_literals;

// ============ 力/力矩轴索引定义 ============
enum { 
  AX_FX=0,        // X方向力
  AX_FY=1,        // Y方向力
  AX_FZ=2,        // Z方向力
  AX_MX=3,        // X方向力矩
  AX_MY=4,        // Y方向力矩
  AX_MZ=5         // Z方向力矩
};

// ============ 控制模式定义 ============
enum { 
  RM_FIXED=0,         // 固定模式：该轴被锁定
  RM_FLOAT=1,         // 浮动模式：该轴自由运动
  RM_SPRING=2,        // 弹簧模式：弹性控制
  RM_MOTION=3,        // 运动模式：沿指定方向运动
  RM_FTRACK=4,        // 力跟踪模式：跟踪目标力
  RM_FTRACK_ADAPT=8   // 自适应力跟踪模式
};




// ========================================= 类定义 =========================================
// 力位混合控制客户端类
// 功能：定期发送力位混合控制参数到机械臂驱动
class ForcePositionClient: public rclcpp::Node
{
private:
  // ============ 成员变量 ============
  // int stage_ = 0;             // 状态机阶段（已注释）
  // bool movejp_ok_ = false;    // 关节空间运动完成标志（已注释）
  // bool setfp_ok_  = false;    // 力位混合设置完成标志（已注释）
  
  // 时间戳（用于时间同步）
  rclcpp::Time t0_;
  
  // 定时器，每20ms触发一次tick()函数
  rclcpp::TimerBase::SharedPtr timer_;

  // 发布器（已注释，暂不使用）
  // rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr pub_movejp_;
  
  // 力位混合控制参数发布器（发送到/rm_driver/set_force_postion_new_cmd话题）
  rclcpp::Publisher<rm_ros_interfaces::msg::Setforcepositionnew>::SharedPtr pub_fp;
  
  // rclcpp::Publisher<rm_ros_interfaces::msg::Forcepositionmove>::SharedPtr pub_movefp_; // （已注释）
  
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_movejp_ok_; // （已注释）
 
  // ============ 私有方法 ============
  // 定时器回调函数，每20ms执行一次
  // 功能：构建力位混合控制参数并发布
  void tick()
  {
    // ============ 力位混合控制策略 ============
    // 应用场景：机械臂接触表面进行力控操作
    // 控制策略：
    //   - X/Y轴（Fx/Fy）：浮动模式，允许水平自由运动
    //   - Z轴（Fz）：力跟踪模式，保持恒定20N压力
    //   - 旋转轴（Mx/My/Mz）：全固定，不允许旋转
    
    // 创建力位混合控制参数消息
    rm_ros_interfaces::msg::Setforcepositionnew fp;
    
    // ============ 基本参数设置 ============
    fp.sensor = 1;  // 使用六维力传感器（1表示使用，0表示不使用）
    fp.mode   = 1;  // 工具坐标系模式（1=工具坐标系，0=基座坐标系）
    
    // ============ 控制模式设置（6个轴：Fx, Fy, Fz, Mx, My, Mz） ============
    // 注意：该消息假设control_mode是一个数组，可根据实际msg定义进行调整
    fp.control_mode[0] = RM_FLOAT;    // Fx轴：浮动模式 - 自由运动，不控制
    fp.control_mode[1] = RM_FLOAT;    // Fy轴：浮动模式 - 自由运动，不控制
    fp.control_mode[2] = RM_FTRACK;   // Fz轴：力跟踪模式 - 维持恒定压力20N
    fp.control_mode[3] = RM_FIXED;    // Mx轴：固定模式 - 锁定，不允许绕X转
    fp.control_mode[4] = RM_FIXED;    // My轴：固定模式 - 锁定，不允许绕Y转
    fp.control_mode[5] = RM_FIXED;    // Mz轴：固定模式 - 锁定，不允许绕Z转

    // ============ 目标力/力矩设置 ============
    fp.desired_force[0] = 0;          // Fx轴目标力 = 0N（无水平力约束）
    fp.desired_force[1] = 0;          // Fy轴目标力 = 0N（无水平力约束）
    fp.desired_force[2] = 20.0f;      // Fz轴目标力 = 20N（垂直向下20N压力）
    fp.desired_force[3] = 0;          // Mx轴目标力矩 = 0Nm（无扭矩）
    fp.desired_force[4] = 0;          // My轴目标力矩 = 0Nm（无扭矩）
    fp.desired_force[5] = 0;          // Mz轴目标力矩 = 0Nm（无扭矩）

    // ============ 速度限制设置 ============
    // 控制各轴在浮动模式下的最大运动速度，防止过快移动
    fp.limit_vel[0] = 0.10f;          // Fx方向最大速度 = 0.1 m/s
    fp.limit_vel[1] = 0.10f;          // Fy方向最大速度 = 0.1 m/s
    fp.limit_vel[2] = 0.02f;          // Fz方向最大速度 = 0.02 m/s（垂直速度受限更严格）
    fp.limit_vel[3] = 0.30f;          // Mx方向最大角速度 = 0.3 rad/s
    fp.limit_vel[4] = 0.30f;          // My方向最大角速度 = 0.3 rad/s
    fp.limit_vel[5] = 0.30f;          // Mz方向最大角速度 = 0.3 rad/s
    
    // ============ 发布控制指令 ============
    pub_fp->publish(fp);              // 通过发布器发送力位混合参数到机械臂驱动

    return;
  }
  
public:
  // ============ 构造函数 ============
  // 初始化ROS2节点，创建发布器和定时器
  ForcePositionClient() : Node("force_position_client")
  {
    // 创建关节空间运动发布器（已注释，暂不使用）
    // pub_movejp_ = create_publisher<rm_ros_interfaces::msg::Movejp>(
    //     "/rm_driver/movej_p_cmd", rclcpp::SystemDefaultsQoS());
   
    // ============ 创建力位混合控制发布器 ============
    // 发布到话题"/rm_driver/set_force_postion_new_cmd"
    // 使用系统默认QoS（服务质量）配置
    pub_fp = create_publisher<rm_ros_interfaces::msg::Setforcepositionnew>(
        "/rm_driver/set_force_postion_new_cmd", 
        rclcpp::SystemDefaultsQoS());

    // 订阅运动完成信号（已注释，暂不使用）
    // sub_movejp_ok_ = create_subscription<std_msgs::msg::Bool>(
    //     "/rm_driver/movej_p_result", 
    //     rclcpp::SystemDefaultsQoS(),
    //     [&](std_msgs::msg::Bool::SharedPtr m){ movejp_ok_ = m->data; });
    
    // ============ 创建定时器 ============
    // 每20毫秒(50Hz)触发一次tick()回调函数
    // 实现周期性的控制指令发送
    timer_ = create_wall_timer(20ms, std::bind(&ForcePositionClient::tick, this));
  }
};



// class ForcePositionControlDemoSub: public rclcpp::Node
// {
//   public:
//     ForcePositionControlDemoSub();                                                                         //构造函数
//     void ForcePositionControl_demo();                                                                   //力位混合运动规划函数
//     void MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                 //结果回调函数
//     void SetForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                        //结果回调函数
//     void MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                                  //结果回调函数
//     void StopForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg);                       //结果回调函数
    
//   private:
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_p_subscription_;                         //声明订阅器
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_subscription_;                           //声明订阅器
    
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_force_postion_subscription_;               //声明订阅器
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_force_postion_subscription_;              //声明订阅器

// };

// class ForcePositionControlDemoPub: public rclcpp::Node
// {
//   public:
//     ForcePositionControlDemoPub();                                                                      //构造函数
//     void ForcePositionControl_demo();                                                                   //力位混合运动规划函数
//     void looppub_timer_callback();                                                                      //move运动规划函数
    
//   private:
//     rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movej_p_publisher_;                    //声明发布器
//     rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;                       //声明发布器
    
//     rclcpp::Publisher<rm_ros_interfaces::msg::Setforceposition>::SharedPtr set_force_postion_publisher_;//声明发布器
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_force_postion_publisher_;                    //声明发布器
//     rclcpp::TimerBase::SharedPtr loop_pub_Timer;                                                        //定时发布器

// };

// /******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
// void ForcePositionControlDemoSub::MoveJPDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     // 将接收到的消息打印出来，显示是否执行成功
//     movej_p_state = true;
//     if(msg->data)
//     {
//         RCLCPP_INFO (this->get_logger(),"*******Movej_p succeeded\n");
//     } else {
//         RCLCPP_ERROR (this->get_logger(),"*******Movej_p Failed\n");
//     }
// }   
// /***********************************************end**************************************************/

// /******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
// void ForcePositionControlDemoSub::MoveLDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     // 将接收到的消息打印出来，显示是否执行成功
//     movel_state = msg->data;
//     if(msg->data)
//     {
//         RCLCPP_INFO (this->get_logger(),"*******MoveL succeeded\n");
//     } else {
//         RCLCPP_ERROR (this->get_logger(),"*******MoveL Failed\n");
//     }
// }   
// /***********************************************end**************************************************/

// /******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
// void ForcePositionControlDemoSub::SetForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     // 将接收到的消息打印出来，显示是否执行成功
//     set_force_postion_state = msg->data;
//     if(msg->data)
//     {
//         RCLCPP_INFO (this->get_logger(),"*******Set Force Postion succeeded\n");
//     } else {
//         RCLCPP_ERROR (this->get_logger(),"*******Set Force Postion Failed\n");
//     }
// }   
// /***********************************************end**************************************************/

// /******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
// void ForcePositionControlDemoSub::StopForcePostionDemo_Callback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//     // 将接收到的消息打印出来，显示是否执行成功
//     stop_force_postion_state = true;
//     if(msg->data)
//     {
//         RCLCPP_INFO (this->get_logger(),"*******Stop Force Postion succeeded\n");
//     } else {
//         RCLCPP_ERROR (this->get_logger(),"*******Stop Force Postion Failed\n");
//     }
// }   
// /***********************************************end**************************************************/

// /*******************************************力位混合运动函数****************************************/
// void ForcePositionControlDemoPub::looppub_timer_callback()
// {
//   //moveJP到达指定位置 
//   if(first_run ==true)
//   {
//     rm_ros_interfaces::msg::Movejp moveJ_P_TargetPose;
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     moveJ_P_TargetPose.pose.position.x = -0.355816;
//     moveJ_P_TargetPose.pose.position.y = -0.000013;
//     moveJ_P_TargetPose.pose.position.z = 0.222814;
//     moveJ_P_TargetPose.pose.orientation.x = 0.995179;
//     moveJ_P_TargetPose.pose.orientation.y = -0.094604;
//     moveJ_P_TargetPose.pose.orientation.z = -0.025721;
//     moveJ_P_TargetPose.pose.orientation.w = 0.002349;
//     moveJ_P_TargetPose.speed = 20;
//     moveJ_P_TargetPose.block = true;
//     this->movej_p_publisher_->publish(moveJ_P_TargetPose);
//     first_run = false;
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   }
//   //开启力位混合 
//   if(movej_p_state==true)                    //等待moveJ_P到达
//   {
//     rm_ros_interfaces::msg::Setforceposition forceposition_data;
//     forceposition_data.sensor = 1;
//     forceposition_data.mode = 0;
//     forceposition_data.direction = 1;
//     forceposition_data.n = 5;
//     // forceposition_data.block = true;
//     this->set_force_postion_publisher_->publish(forceposition_data);
//     movej_p_state = false;
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//   }
//   //moveL运动 
//   if(set_force_postion_state==true)
//   {
//     rm_ros_interfaces::msg::Movel moveL_TargetPose;
//     moveL_TargetPose.pose.position.x = -0.255816;
//     moveL_TargetPose.pose.position.y = -0.000013;
//     moveL_TargetPose.pose.position.z = 0.222814;
//     moveL_TargetPose.pose.orientation.x = 0.995179;
//     moveL_TargetPose.pose.orientation.y = -0.094604;
//     moveL_TargetPose.pose.orientation.z = -0.025721;
//     moveL_TargetPose.pose.orientation.w = 0.002349;
//     moveL_TargetPose.speed = 20;
//     moveL_TargetPose.block = true;
//     this->movel_publisher_->publish(moveL_TargetPose);
//     set_force_postion_state = false;
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   }
//   //停止力位混合 
//   if(movel_state==true)                     //等待movel到达
//   {
//     std_msgs::msg::Bool stop_force_postion_data;
//     stop_force_postion_data.data = true;
//     this->stop_force_postion_publisher_->publish(stop_force_postion_data);
//     movel_state = false;
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//   }
//   if(stop_force_postion_state==true)
//   {
//     RCLCPP_INFO (this->get_logger(),"*******All step run over\n");
//     stop_force_postion_state = false;
//   }
// }
/***********************************************end**************************************************/

/***********************************构造函数，初始化发布器订阅器****************************************/
// ForcePositionControlDemoPub::ForcePositionControlDemoPub():rclcpp::Node("Force_Position_Control_pub_node")
// {
//   movej_p_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movejp>("/rm_driver/movej_p_cmd", rclcpp::ParametersQoS());
//   movel_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movel>("/rm_driver/movel_cmd", rclcpp::ParametersQoS());
//   set_force_postion_publisher_ = this->create_publisher<rm_ros_interfaces::msg::Setforceposition>("/rm_driver/set_force_postion_cmd", rclcpp::ParametersQoS());
//   stop_force_postion_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/rm_driver/stop_force_postion_cmd", rclcpp::ParametersQoS());
//   loop_pub_Timer = this->create_wall_timer(std::chrono::milliseconds(100), 
//         std::bind(&ForcePositionControlDemoPub::looppub_timer_callback,this));
//   std::this_thread::sleep_for(std::chrono::milliseconds(2000));
// }
// /***********************************************end**************************************************/

// /***********************************构造函数，初始化发布器订阅器****************************************/
// ForcePositionControlDemoSub::ForcePositionControlDemoSub():rclcpp::Node("Force_Position_Control_sub_node")
// {
//   movej_p_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_p_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::MoveJPDemo_Callback, this,_1));
//   movel_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movel_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::MoveLDemo_Callback, this,_1));
//   set_force_postion_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/set_force_postion_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::SetForcePostionDemo_Callback, this,_1));
//   stop_force_postion_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/stop_force_postion_result", rclcpp::ParametersQoS(), std::bind(&ForcePositionControlDemoSub::StopForcePostionDemo_Callback, this,_1));
//   std::this_thread::sleep_for(std::chrono::milliseconds(3000));
// }
/***********************************************end**************************************************/

// ========================================= 主函数 =========================================
// 程序入口点
int main(int argc, char** argv)
{
  // ============ ROS2初始化 ============
  // 初始化ROS2系统，处理命令行参数
  rclcpp::init(argc, argv);
  
  // ============ 替代实现方案（已注释） ============
  // 该方案使用单线程执行器管理多个节点（发布器/订阅器分开）
  // rclcpp::executors::SingleThreadedExecutor executor;
  // auto node_sub = std::make_shared<ForcePositionControlDemoSub>();    // 订阅器节点
  // auto node_pub = std::make_shared<ForcePositionControlDemoPub>();    // 发布器节点
  // executor.add_node(node_pub);
  // executor.add_node(node_sub);
  // executor.spin();  // 阻塞运行
  
  // ============ 当前实现方案 ============
  // 创建ForcePositionClient节点实例，并进入事件循环
  // 节点将持续运行，每20ms执行一次tick()函数发送控制指令
  rclcpp::spin(std::make_shared<ForcePositionClient>());
  
  // ============ 程序清理 ============
  // ROS2系统关闭，释放资源
  rclcpp::shutdown();
  
  return 0;
}
