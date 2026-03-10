//
// Created by ubuntu on 24-7-11.
//
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rm_ros_interfaces/msg/movejp.hpp"
#include "rm_ros_interfaces/msg/setforceposition.hpp"
#include "rm_ros_interfaces/msg/forcepositionmove.hpp"
#include "rm_driver/rm_define.h"
#include "rm_driver/rm_interface.h"
#include "rm_driver/rm_interface_global.h"
#include "rm_driver/rm_service.h"
#include "rm_driver/rm_driver.h"




using namespace std::chrono_literals;
enum { AX_FX=0, AX_FY=1, AX_FZ=2, AX_MX=3, AX_MY=4, AX_MZ=5 };
enum { RM_FIXED=0, RM_FLOAT=1, RM_SPRING=2, RM_MOTION=3, RM_FTRACK=4, RM_FTRACK_ADAPT=8 };




/****************************************创建类************************************/ 
class ForcePositionClient: public rclcpp::Node
{
private:
  /* data */
  rm_robot_handle *handle_;
  int stage_ = 0;
  bool movejp_ok_ = false;
  bool setfp_ok_  = false;
  bool stop_ok_   = false;

  rclcpp::Time t0_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr pub_movejp_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_start_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Forcepositionmove>::SharedPtr pub_movefp_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_stop_;
 

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_movejp_ok_;
 
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_ok_;
  void tick()
  {
    if (stage_ == 0) {
      // 1) 先 MoveJ_P 到接触前位置（离工件稍微留一点距离）
      rm_ros_interfaces::msg::Movejp cmd;
      cmd.pose.position.x = 0.355816;
      cmd.pose.position.y = 0.000013;
      cmd.pose.position.z =  0.222814;
      cmd.pose.orientation.x = 0.995179;
      cmd.pose.orientation.y = -0.094604;
      cmd.pose.orientation.z = -0.025721;
      cmd.pose.orientation.w = 0.002349;
      cmd.speed = 20;
      cmd.block = true;
      pub_movejp_->publish(cmd);

      stage_ = 1;
      RCLCPP_INFO(get_logger(), "Stage0: sent MoveJ_P");
      return;
    }

    if (stage_ == 1) {
      if (!movejp_ok_) return;

      // 2) 设置力位混合参数：工具坐标系下 Fz 恒力 20N，Fx/Fy 运动，姿态固定
      rm_force_position_t fp;
      fp.sensor = 1;  // 六维力
      fp.mode   = 1;  // 工具坐标系

      // === TODO：如果你 msg 字段不是数组，改成你自己的字段 ===
      int modes[6]   = {RM_FLOAT, RM_FLOAT, RM_FTRACK, RM_FIXED, RM_FIXED, RM_FIXED};
      float f[6]     = {0.f, 0.f, 20.f, 0.f, 0.f, 0.f};
      float vlim[6]  = {0.10f, 0.10f, 0.02f, 0.30f, 0.30f, 0.30f};
      std::copy(std::begin(modes),  std::end(modes),  fp.control_mode);
      std::copy(std::begin(f),      std::end(f),      fp.desired_force);
      std::copy(std::begin(vlim),   std::end(vlim),   fp.limit_vel);
      int res;
      res= rm_set_force_position_new(handle_, fp);
      if(res==0)stage_ = 2;

      RCLCPP_INFO(get_logger(), "Stage1: sent SetForcePosition");
      return;
    }

    if (stage_ == 2) {
      

      // 3) 开启透传
      std_msgs::msg::Empty e;
      pub_start_->publish(e);

      t0_ = now();
      stage_ = 3;
      RCLCPP_INFO(get_logger(), "Stage2: sent StartForcePositionMove");
      return;
    }


    if (stage_ == 4) {
      if (stop_ok_) {
        RCLCPP_INFO(get_logger(), "All done.");
        stage_ = 5;
      }
    }
  } 
public:
  ForcePositionClient() : Node("force_position_client_demo")
  {
    pub_movejp_ = create_publisher<rm_ros_interfaces::msg::Movejp>("/rm_driver/movej_p_cmd", rclcpp::SystemDefaultsQoS());
   
    pub_start_  = create_publisher<std_msgs::msg::Empty>("/rm_driver/start_force_position_move_cmd", rclcpp::SystemDefaultsQoS());
   
    pub_stop_   = create_publisher<std_msgs::msg::Empty>("/rm_driver/stop_force_position_move_cmd", rclcpp::SystemDefaultsQoS());

    sub_movejp_ok_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/movej_p_result", rclcpp::SystemDefaultsQoS(),
      [&](std_msgs::msg::Bool::SharedPtr m){ movejp_ok_ = m->data; });



    sub_stop_ok_ = create_subscription<std_msgs::msg::Bool>(
      "/rm_driver/stop_force_position_move_result", rclcpp::SystemDefaultsQoS(),
      [&](std_msgs::msg::Bool::SharedPtr m){ stop_ok_ = m->data; });

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

/******************************************************主函数*********************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::executors::SingleThreadedExecutor executor;
  // auto node_sub = std::make_shared<ForcePositionControlDemoSub>();
  // auto node_pub = std::make_shared<ForcePositionControlDemoPub>();
  // executor.add_node(node_pub);
  // executor.add_node(node_sub);
  
  // executor.spin();
  rclcpp::spin(std::make_shared<ForcePositionClient>());
  rclcpp::shutdown();
  return 0;
}
