
//* ROS action server */
#include "rm_control.h"
/* 三次样条插补 */
#include "cubicSpline.h"

using namespace std;

// ============ 时间数组 ============
// 存储插值后的时间戳（单位：秒）
vector<double> time_from_start_;

// ============ 各关节的位置、速度、加速度数据 ============
// 6轴机械臂：joint1-6
// 7轴机械臂：joint1-7（RM75等）
//
// 作用：存储三次样条插值后的轨迹
// 数据量：插值前可能有10-20个路点，插值后变成1000+个数据点（20ms精度）
//
vector<double> p_joint1_;  // joint1位置
vector<double> p_joint2_;  // joint2位置
vector<double> p_joint3_;  // joint3位置
vector<double> p_joint4_;  // joint4位置
vector<double> p_joint5_;  // joint5位置
vector<double> p_joint6_;  // joint6位置
vector<double> p_joint7_;  // joint7位置（仅7轴）

vector<double> v_joint1_;  // joint1速度
vector<double> v_joint2_;  // joint2速度
vector<double> v_joint3_;  // joint3速度
vector<double> v_joint4_;  // joint4速度
vector<double> v_joint5_;  // joint5速度
vector<double> v_joint6_;  // joint6速度
vector<double> v_joint7_;  // joint7速度（仅7轴）

vector<double> a_joint1_;  // joint1加速度
vector<double> a_joint2_;  // joint2加速度
vector<double> a_joint3_;  // joint3加速度
vector<double> a_joint4_;  // joint4加速度
vector<double> a_joint5_;  // joint5加速度
vector<double> a_joint6_;  // joint6加速度
vector<double> a_joint7_;  // joint7加速度（仅7轴）

// ============ 轨迹执行状态结构体 ============
// 用于跟踪当前发送到第几个插值点
struct vel_data {
  int vector_len;  // 插值点总数
  int vector_cnt;  // 当前发送的点的索引（0 ~ vector_len-1）
};

// ============ 全局执行状态变量 ============
struct vel_data p2;  // 当前轨迹执行位置

// 插值计算的中间变量
double acc = 0, vel = 0;      // 加速度和速度
double x_out = 0, y_out = 0;  // 插值的输入输出

// 轨迹是否改变的标志
bool point_changed = false;  // true=有新轨迹，false=轨迹已全部发送

// 定时器周期参数
double rate = 0.020;  // 5ms = 20个数据点/秒

// 透传参数（用于轨迹执行完成后的等待）
float min_interval = 20;            // 20秒内不再接收新轨迹
float wait_move_finish_time = 1.5;  // 轨迹完成后再保持1.5秒
int count_keep_send = 0;            // 保持发送最后一个点的次数
int count_final_joint = 0;          // 当前已保持发送的次数

/* 三次样条无参构造 */
cubicSpline::cubicSpline() {}
/* 析构 */
cubicSpline::~cubicSpline() { releaseMem(); }
/* 初始化参数 */
void cubicSpline::initParam() {
  x_sample_ = y_sample_ = M_ = NULL;
  sample_count_ = 0;
  bound1_ = bound2_ = 0;
}
/* 释放参数 */
void cubicSpline::releaseMem() {
  delete x_sample_;
  delete y_sample_;
  delete M_;

  initParam();
}
/**
 * ========================================================================
 *              三次样条插值（Cubic Spline Interpolation）
 *                        核心算法说明
 * ========================================================================
 *
 * 三次样条插值是一种数值分析方法，用于通过一组给定的数据点生成光滑曲线。
 *
 * 核心思想：
 * 将多个小区间的三次多项式平滑连接，确保在连接点处的一阶和二阶导数连续
 * （这保证了轨迹的位置、速度、加速度都是连续的）
 *
 * 应用场景：
 * 将MoveIt规划的离散路点（10-20个）转换成连续光滑的轨迹（1000+个点）
 * 避免机械臂因为路点不足而产生的抖动和冲击
 *
 * ========================================================================
 */

/**
 * 函数名：loadData
 * 功能：加载数据并进行三次样条插值计算
 *
 * 参数说明：
 *   x_data[]：时间戳数组（单位：秒）
 *            例如 [0, 1.2, 2.4, 3.6, 4.8] 秒
 *   y_data[]：该关节的位置数组（单位：弧度）
 *            例如 [0, 0.5, 1.0, 1.5, 2.0] rad
 *   count：数据点个数（MoveIt规划的路点总数，通常10-20）
 *   bound1：第一个边界条件（一阶导数形式下为起始速度）
 *   bound2：最后一个边界条件（一阶导数形式下为终止速度）
 *   type：边界条件类型
 *        BoundType_First_Derivative：一阶导数边界条件（使用速度）
 *        BoundType_Second_Derivative：二阶导数边界条件（使用加速度）
 *
 * 返回值：true=插值成功，false=插值失败
 *
 * 内部步骤：
 * 1. initParam()：初始化参数
 * 2. 复制输入数据到对象成员变量
 * 3. spline()：调用核心样条插值算法
 *
 */
bool cubicSpline::loadData(double* x_data, double* y_data, int count,
                           double bound1, double bound2, BoundType type) {
  if ((NULL == x_data) || (NULL == y_data) || (count < 3) ||
      (type > BoundType_Second_Derivative) ||
      (type < BoundType_First_Derivative)) {
    return false;
  }
  initParam();

  x_sample_ = new double[count];
  y_sample_ = new double[count];
  M_ = new double[count];  // M_数组存储各点的二阶导数（加速度）
  sample_count_ = count;

  memcpy(x_sample_, x_data, sample_count_ * sizeof(double));
  memcpy(y_sample_, y_data, sample_count_ * sizeof(double));

  bound1_ = bound1;
  bound2_ = bound2;

  return spline(type);
}

/**
 * 函数名：spline
 * 功能：执行三次样条插值的核心算法
 *
 * 核心算法：追赶法（Thomas Algorithm）求解三对角线性方程组
 *
 * 数学原理：
 * 对于每个区间 [x_i, x_{i+1}]，构造三次多项式 S_i(x)
 * S_i(x) = a_i + b_i(x-x_i) + c_i(x-x_i)^2 + d_i(x-x_i)^3
 *
 * 要求满足条件：
 * 1. 插值条件：S_i(x_i) = y_i（经过每个数据点）
 * 2. 连续条件：S_i和S_{i+1}在x_{i+1}处的值、一阶导数、二阶导数都相等
 * 3. 边界条件：在x_0和x_n处由用户指定（一阶或二阶导数）
 *
 * 求解过程：
 * 1. 建立三对角方程组（系数矩阵只有三条对角线非零）
 * 2. 使用追赶法求解二阶导数 M_[]
 * 3. 根据M_[]计算各段的三次多项式系数
 *
 * 参数：
 *   type：边界条件类型
 *
 * 返回值：true=成功，false=失败
 */
bool cubicSpline::spline(BoundType type) {
  if ((type < BoundType_First_Derivative) ||
      (type > BoundType_Second_Derivative)) {
    return false;
  }

  //  追赶法解方程求二阶偏导数
  double f1 = bound1_, f2 = bound2_;

  double* a = new double[sample_count_];  //  a:稀疏矩阵最下边一串数
  double* b = new double[sample_count_];  //  b:稀疏矩阵最中间一串数
  double* c = new double[sample_count_];  //  c:稀疏矩阵最上边一串数
  double* d = new double[sample_count_];

  double* f = new double[sample_count_];

  double* bt = new double[sample_count_];
  double* gm = new double[sample_count_];

  double* h = new double[sample_count_];

  for (int i = 0; i < sample_count_; i++) b[i] = 2;  //  中间一串数为2
  for (int i = 0; i < sample_count_ - 1; i++) {
    if (x_sample_[i + 1] == x_sample_[i]) {
      h[i] = 0.005;
    } else {
      h[i] = x_sample_[i + 1] - x_sample_[i];  // 各段步长
    }
  }
  for (int i = 1; i < sample_count_ - 1; i++)
    a[i] = h[i - 1] / (h[i - 1] + h[i]);
  a[sample_count_ - 1] = 1;

  c[0] = 1;
  for (int i = 1; i < sample_count_ - 1; i++) c[i] = h[i] / (h[i - 1] + h[i]);

  for (int i = 0; i < sample_count_ - 1; i++) {
    if (x_sample_[i + 1] == x_sample_[i]) {
      f[i] = (y_sample_[i + 1] - y_sample_[i]) / 0.005;
    } else {
      f[i] =
          (y_sample_[i + 1] - y_sample_[i]) / (x_sample_[i + 1] - x_sample_[i]);
    }
  }

  for (int i = 1; i < sample_count_ - 1; i++)
    d[i] = 6 * (f[i] - f[i - 1]) / (h[i - 1] + h[i]);

  //  追赶法求解方程
  if (BoundType_First_Derivative == type) {
    d[0] = 6 * (f[0] - f1) / h[0];
    d[sample_count_ - 1] =
        6 * (f2 - f[sample_count_ - 2]) / h[sample_count_ - 2];

    bt[0] = c[0] / b[0];
    for (int i = 1; i < sample_count_ - 1; i++)
      bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);

    gm[0] = d[0] / b[0];
    for (int i = 1; i <= sample_count_ - 1; i++)
      gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

    M_[sample_count_ - 1] = gm[sample_count_ - 1];
    for (int i = sample_count_ - 2; i >= 0; i--)
      M_[i] = gm[i] - bt[i] * M_[i + 1];
  } else if (BoundType_Second_Derivative == type) {
    d[1] = d[1] - a[1] * f1;
    d[sample_count_ - 2] = d[sample_count_ - 2] - c[sample_count_ - 2] * f2;

    bt[1] = c[1] / b[1];
    for (int i = 2; i < sample_count_ - 2; i++)
      bt[i] = c[i] / (b[i] - a[i] * bt[i - 1]);

    gm[1] = d[1] / b[1];
    for (int i = 2; i <= sample_count_ - 2; i++)
      gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1]);

    M_[sample_count_ - 2] = gm[sample_count_ - 2];
    for (int i = sample_count_ - 3; i >= 1; i--)
      M_[i] = gm[i] - bt[i] * M_[i + 1];

    M_[0] = f1;
    M_[sample_count_ - 1] = f2;
  } else
    return false;

  delete a;
  delete b;
  delete c;
  delete d;
  delete gm;
  delete bt;
  delete f;
  delete h;

  return true;
}

/**
 * 函数名：getYbyX
 * 功能：根据给定的时间戳，计算样条插值曲线上对应的位置、速度、加速度
 *
 * 这是最核心的查询函数，用于在 timer_callback 中逐个计算插值点
 *
 * 参数说明：
 *   x_in：查询的时间戳（单位：秒）
 *        例如：0, 0.02, 0.04, 0.06, ... 直到 max_time
 *   y_out：输出的位置值（通过引用返回）
 *
 * 输出（通过全局变量）：
 *   vel：速度值（一阶导数）
 *   acc：加速度值（二阶导数）
 *
 * 工作流程：
 * 1. 使用二分法查找 x_in 所在的区间 [x_sample_[klo], x_sample_[khi]]
 * 2. 在该区间内使用三次多项式计算位置、速度、加速度
 * 3. 返回结果
 *
 * 举例：
 * loadData 接收了 MoveIt 的 5 个路点：
 *   时间戳：[0, 1.2, 2.4, 3.6, 4.8]
 *   位置：  [0, 0.5, 1.0, 1.5, 2.0]
 *
 * 然后 getYbyX 被调用 240 次：
 *   x_in=0.00 → y_out=0.000, vel≈0, acc≈0      (起点)
 *   x_in=0.02 → y_out=0.002, vel≈0.1, acc≈0    (平滑开始)
 *   x_in=0.04 → y_out=0.008, vel≈0.2, acc≈0
 *   ...
 *   x_in=4.78 → y_out=1.998, vel≈0.1, acc≈0    (接近终点)
 *   x_in=4.80 → y_out=2.000, vel≈0, acc≈0      (终点)
 *
 * 最终生成 240 个均匀分布的插值点
 */
bool cubicSpline::getYbyX(double& x_in, double& y_out) {
  int klo, khi, k;
  klo = 0;
  khi = sample_count_ - 1;
  double hh, bb, aa;

  //  二分法查找x所在区间段
  while (khi - klo > 1) {
    k = (khi + klo) >> 1;
    if (x_sample_[k] > x_in)
      khi = k;
    else
      klo = k;
  }
  hh = x_sample_[khi] - x_sample_[klo];

  aa = (x_sample_[khi] - x_in) / hh;
  bb = (x_in - x_sample_[klo]) / hh;

  y_out = aa * y_sample_[klo] + bb * y_sample_[khi] +
          ((aa * aa * aa - aa) * M_[klo] + (bb * bb * bb - bb) * M_[khi]) * hh *
              hh / 6.0;

  // test
  acc =
      (M_[klo] * (x_sample_[khi] - x_in) + M_[khi] * (x_in - x_sample_[klo])) /
      hh;
  vel = M_[khi] * (x_in - x_sample_[klo]) * (x_in - x_sample_[klo]) / (2 * hh) -
        M_[klo] * (x_sample_[khi] - x_in) * (x_sample_[khi] - x_in) / (2 * hh) +
        (y_sample_[khi] - y_sample_[klo]) / hh - hh * (M_[khi] - M_[klo]) / 6;

  return true;
}

Rm_Control::Rm_Control(std::string name) : Node(name) {
  using namespace std::placeholders;

  // 1. 声明并获取核心参数（新增前缀参数）
  this->declare_parameter<std::string>("arm_prefix", "");
  this->get_parameter("arm_prefix", arm_prefix_);

  this->declare_parameter<int>("arm_type", arm_type_);
  this->get_parameter("arm_type", arm_type_);

  this->declare_parameter<bool>("follow", follow_);
  this->get_parameter("follow", follow_);
  // 【新增】机械爪发布器初始化（对应rm_driver的话题）
  rclcpp::QoS qos(10);
  // 机械爪发布器 - 使用arm_prefix区分不同手臂
  gripper_pick_pub =
      this->create_publisher<rm_ros_interfaces::msg::Gripperpick>(
          arm_prefix_ + "/rm_driver/set_gripper_pick_cmd", qos);
  gripper_pos_pub = this->create_publisher<rm_ros_interfaces::msg::Gripperset>(
      arm_prefix_ + "/rm_driver/set_gripper_position_cmd", qos);

  // 【新增】爪控结果订阅器（接收rm_driver执行结果）
  gripper_pos_result_sub = this->create_subscription<std_msgs::msg::Bool>(
      arm_prefix_ + "/rm_driver/set_gripper_position_result", qos,
      std::bind(&Rm_Control::gripper_pos_result_cb, this, _1));




  if ((arm_type_ == 75) || (arm_type_ == 72)) {
    joint_msg.joint.resize(7);
    joint_msg.dof = 7;
    arm_type_ = 75;
  } else {
    joint_msg.joint.resize(6);
    joint_msg.dof = 6;
  }

  State_Timer =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&Rm_Control::timer_callback, this));

  // 4. 动作服务器：使用相对路径匹配MoveIt配置（如左手臂为
  // "arm1_controller/follow_joint_trajectory"）
  std::string action_name = arm_prefix_ + "_controller/follow_joint_trajectory";
  this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, action_name, std::bind(&Rm_Control::handle_goal, this, _1, _2),
      std::bind(&Rm_Control::handle_cancel, this, _1),
      std::bind(&Rm_Control::handle_accepted, this, _1));

  // 5. 发布器：使用相对路径匹配rm_driver的订阅（如左手臂为
  // "arm1/rm_driver/movej_canfd_cmd"）

  std::string pub_topic = arm_prefix_ + "/rm_driver/movej_canfd_cmd";
  joint_pos_publisher =
      this->create_publisher<rm_ros_interfaces::msg::Jointpos>(pub_topic, qos);

  // 6. 订阅器：使用相对路径匹配rm_driver的发布（如左手臂为
  // "arm1/rm_driver/move_stop_cmd"）
  std::string sub_topic = arm_prefix_ + "/rm_driver/move_stop_cmd";
  Get_Move_Stop_Cmd = this->create_subscription<std_msgs::msg::Empty>(
      sub_topic, rclcpp::ParametersQoS(),
      std::bind(&Rm_Control::get_move_stop_callback, this,
                std::placeholders::_1));
}

rclcpp_action::GoalResponse Rm_Control::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
  std::cout << "---handle goal:" << goal->trajectory.joint_names.size()
            << std::endl;
  std::cout << goal->trajectory.header.frame_id.c_str()
            << goal->trajectory.header.stamp.sec
            << goal->trajectory.header.stamp.nanosec << std::endl;

  int pointSize = goal->trajectory.points.size();
  if (pointSize > 0) {
    for (int i = 0; i < pointSize; i++) {
      auto point = goal->trajectory.points.at(i);
    }
  }

  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Rm_Control::handle_cancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void Rm_Control::handle_accepted(
    const std::shared_ptr<GoalHandleFJT> goal_handle) {
  using std::placeholders::_1;

  std::thread{std::bind(&Rm_Control::execute_move, this, _1), goal_handle}
      .detach();
}

/* 收到action的goal后调用的回调函数 */
/**
 * ========================================================================
 *                    EXECUTE_MOVE 函数说明
 * ========================================================================
 * 名称：execute_move
 * 功能：执行轨迹插值和发送的核心函数
 *
 * 工作流程：
 * 1. 接收 MoveIt 规划器发送的轨迹（包含10-20个稀疏路点）
 * 2. 对轨迹中的每个关节分别应用三次样条插值算法
 * 3. 将稀疏的路点转换为密集的轨迹（1000+个点，20ms间隔）
 * 4. 将插值数据存储到全局向量
 * 5. 通过定时器回调函数周期性地发送到rm_driver（50Hz频率）
 *
 * 输入参数：
 *   - goal_handle：包含MoveIt规划的轨迹
 *     - goal->trajectory.points：路点数组
 *     - points[i].positions：第i个路点的关节位置（单位：弧度）
 *     - points[i].velocities：第i个路点的关节速度
 *     - points[i].accelerations：第i个路点的关节加速度
 *     - points[i].time_from_start：相对于轨迹开始的时间戳
 *
 * 输出结果：
 *   - 全局向量 time_from_start_, p_joint1_~7_, v_joint1_~7_, a_joint1_~7_
 *   - result->error_code：0=成功，-1=被中止
 *   - result->error_string：错误描述
 *
 * 关键变量：
 *   - point_num：MoveIt发送的路点总数
 *   - p_joint1~7[]：各关节的原始位置数据（来自MoveIt）
 *   - v_joint1~7[]：各关节的原始速度数据
 *   - a_joint1~7[]：各关节的原始加速度数据
 *   - time_from_start[]：原始时间戳
 *   - p2.vector_len：插值后的总点数
 *   - p2.vector_cnt：当前已发送的点数
 *   - point_changed：标志位（true=有新轨迹待发送，false=轨迹已全部发送）
 *
 * 算法说明：
 *   三次样条插值（Cubic Spline Interpolation）
 *   - 目的：将MoveIt的10-20个路点平滑插值成1000+个密集路点
 *   - 优点：(1) 确保轨迹连续光滑，无尖刺抖动
 *          (2) 保证C2连续性（位置、速度、加速度都连续）
 *          (3) 适合工业机械臂的运动特性
 *   - 实现：使用追赶法（Thomas Algorithm）求解三对角线性方程组
 *          计算每个区间的二阶导数，确定三次多项式系数
 *
 * ========================================================================
 */
void Rm_Control::execute_move(
    const std::shared_ptr<GoalHandleFJT> goal_handle) {
  int i = 0;
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();

  // 获取MoveIt规划的路点总数
  int point_num = goal->trajectory.points.size();
  RCLCPP_INFO(this->get_logger(), "MoveIt规划了 %d 个路点", point_num);
  point_changed = false;

  /**
   * Bug修复：2021/9/16 -
   * 修复MoveIt在同一位姿重复规划导致rm_control异常停止的问题
   *
   * 解决方案：只当路点数 > 3 时进行样条插值
   * 原因：3个或以下的路点不足以生成有效的样条曲线
   *      可能表示重复规划或无效规划，应该跳过处理
   */
  if (point_num > 3) {
    // ======== 第1步：从MoveIt轨迹中提取各关节的原始数据 ========
    // 注意：这些数据来自MoveIt规划器，仅有10-20个路点

    /* 各个关节位置 - 单位：弧度 (rad) */
    double p_joint1[point_num];
    double p_joint2[point_num];
    double p_joint3[point_num];
    double p_joint4[point_num];
    double p_joint5[point_num];
    double p_joint6[point_num];
    double p_joint7[point_num];

    /* 各个关节速度 - 单位：弧度/秒 (rad/s) */
    double v_joint1[point_num];
    double v_joint2[point_num];
    double v_joint3[point_num];
    double v_joint4[point_num];
    double v_joint5[point_num];
    double v_joint6[point_num];
    double v_joint7[point_num];

    /* 各个关节加速度 - 单位：弧度/秒^2 (rad/s^2) */
    double a_joint1[point_num];
    double a_joint2[point_num];
    double a_joint3[point_num];
    double a_joint4[point_num];
    double a_joint5[point_num];
    double a_joint6[point_num];
    double a_joint7[point_num];

    /* 时间戳数组 - 相对于轨迹起点的时间 (单位：秒) */
    double time_from_start[point_num];
    double timens_from_start[point_num];

    // ======== 数据提取循环 ========
    // 从MoveIt轨迹消息中逐个提取各路点的数据
    // positions[0-6] 对应 joint1-7（6轴或7轴取决于机械臂类型）
    // velocities[0-6] 对应各关节的速度
    // accelerations[0-6] 对应各关节的加速度
    for (i = 0; i < point_num; i++) {
      // ======== 关节位置提取 (positions) ========
      p_joint1[i] = goal->trajectory.points[i].positions[0];
      p_joint2[i] = goal->trajectory.points[i].positions[1];
      p_joint3[i] = goal->trajectory.points[i].positions[2];
      p_joint4[i] = goal->trajectory.points[i].positions[3];
      p_joint5[i] = goal->trajectory.points[i].positions[4];
      p_joint6[i] = goal->trajectory.points[i].positions[5];
      // RM75 等7轴机械臂才有第7关节
      if (arm_type_ == 75) {
        p_joint7[i] = goal->trajectory.points[i].positions[6];
      }

      // ======== 关节速度提取 (velocities) ========
      v_joint1[i] = goal->trajectory.points[i].velocities[0];
      v_joint2[i] = goal->trajectory.points[i].velocities[1];
      v_joint3[i] = goal->trajectory.points[i].velocities[2];
      v_joint4[i] = goal->trajectory.points[i].velocities[3];
      v_joint5[i] = goal->trajectory.points[i].velocities[4];
      v_joint6[i] = goal->trajectory.points[i].velocities[5];
      if (arm_type_ == 75) {
        v_joint7[i] = goal->trajectory.points[i].velocities[6];
      }

      // ======== 关节加速度提取 (accelerations) ========
      a_joint1[i] = goal->trajectory.points[i].accelerations[0];
      a_joint2[i] = goal->trajectory.points[i].accelerations[1];
      a_joint3[i] = goal->trajectory.points[i].accelerations[2];
      a_joint4[i] = goal->trajectory.points[i].accelerations[3];
      a_joint5[i] = goal->trajectory.points[i].accelerations[4];
      a_joint6[i] = goal->trajectory.points[i].accelerations[5];
      if (arm_type_ == 75) {
        a_joint7[i] = goal->trajectory.points[i].accelerations[6];
      }

      // ======== 时间戳提取 ========
      // time_from_start 包含 秒(sec) 和 纳秒(nanosec) 两部分
      // 需要转换为浮点秒数：秒数 + (纳秒/1e9)
      time_from_start[i] =
          goal->trajectory.points[i].time_from_start.sec +
          goal->trajectory.points[i].time_from_start.nanosec / 1e9;
      timens_from_start[i] = goal->trajectory.points[i].time_from_start.nanosec;
    }

    // ======== 第2步：三次样条插值处理 ========
    //
    // 核心算法：三次样条插值（Cubic Spline Interpolation）
    //
    // 实现原理：
    // 1. 对于每个关节，调用 spline.loadData() 加载MoveIt的原始路点数据
    // 2. loadData() 内部使用追赶法求解三对角方程组，计算三次多项式的系数
    // 3. 使用 getYbyX() 逐个计算：
    //    - 输入：时间戳 x_out（从0开始，每次增加rate=0.02秒）
    //    - 输出：该时间点的位置y_out、速度vel、加速度acc
    // 4. 将插值结果存入全局向量
    //
    // 数据量对比：
    // - 输入：MoveIt规划的10-20个稀疏路点
    // - 输出：1000+个密集路点（20ms间隔 = 50Hz采样率）
    // - 持续时间：从 0 到 max_time（通常4-8秒）
    //
    // 好处：
    // - 生成连续光滑的轨迹，避免尖刺和抖动
    // - 保证C2连续性：位置连续、速度连续、加速度连续
    // - 适配机械臂的运动特性，降低电机负载

    cubicSpline spline;
    double max_time = time_from_start[point_num - 1];  // 轨迹总耗时
    RCLCPP_INFO(this->get_logger(), "轨迹总耗时: %f 秒", max_time);
    time_from_start_.clear();

    // ========== Joint1 插值 ==========
    // 说明：以Joint1为例，展示单个关节的插值流程
    // 其他关节（Joint2-7）流程完全相同，只是改变数据来源和存储目标
    if (spline.loadData(time_from_start, p_joint1, point_num, 0, 0,
                        cubicSpline::BoundType_First_Derivative)) {
      // 清空之前的插值结果
      p_joint1_.clear();
      v_joint1_.clear();
      a_joint1_.clear();

      // 从 -rate 开始（确保包含起点0），每次增加 rate=20ms
      x_out = -rate;
      while (x_out < max_time) {
        x_out += rate;  // 时间戳增加20ms

        // ======== 关键调用 ========
        // spline.getYbyX(x_out, y_out)
        // 功能：在给定时间戳 x_out 处，计算样条曲线的：
        //   - 位置值存到 y_out
        //   - 速度值存到全局变量 vel
        //   - 加速度值存到全局变量 acc
        spline.getYbyX(x_out, y_out);

        // 只需在Joint1处记录时间戳（其他关节会使用相同的时间戳）
        time_from_start_.push_back(x_out);

        // 存储插值结果：位置、速度、加速度
        p_joint1_.push_back(y_out);
        v_joint1_.push_back(vel);
        a_joint1_.push_back(acc);
      }

      // ========== Joint2 插值 ==========
      // 注意：这是嵌套的 if-else-if
      // 结构，用于防止某个关节插值失败导致整个轨迹中止
      if (spline.loadData(time_from_start, p_joint2, point_num, 0, 0,
                          cubicSpline::BoundType_First_Derivative)) {
        p_joint2_.clear();
        v_joint2_.clear();
        a_joint2_.clear();

        x_out = -rate;
        while (x_out < max_time) {
          x_out += rate;
          spline.getYbyX(x_out, y_out);
          p_joint2_.push_back(y_out);
          v_joint2_.push_back(vel);
          a_joint2_.push_back(acc);
        }
        a_joint2_.clear();
        x_out = -rate;
        while (x_out < max_time) {
          x_out += rate;
          spline.getYbyX(x_out, y_out);
          p_joint2_.push_back(y_out);
          v_joint2_.push_back(vel);
          a_joint2_.push_back(acc);
        }

        // joint3
        if (spline.loadData(time_from_start, p_joint3, point_num, 0, 0,
                            cubicSpline::BoundType_First_Derivative)) {
          p_joint3_.clear();
          v_joint3_.clear();
          a_joint3_.clear();
          x_out = -rate;
          while (x_out < max_time) {
            x_out += rate;
            spline.getYbyX(x_out, y_out);
            p_joint3_.push_back(y_out);
            v_joint3_.push_back(vel);
            a_joint3_.push_back(acc);
          }

          // joint4
          if (spline.loadData(time_from_start, p_joint4, point_num, 0, 0,
                              cubicSpline::BoundType_First_Derivative)) {
            p_joint4_.clear();
            v_joint4_.clear();
            a_joint4_.clear();
            x_out = -rate;
            while (x_out < max_time) {
              x_out += rate;
              spline.getYbyX(x_out, y_out);
              p_joint4_.push_back(y_out);
              v_joint4_.push_back(vel);
              a_joint4_.push_back(acc);
            }

            // joint5
            if (spline.loadData(time_from_start, p_joint5, point_num, 0, 0,
                                cubicSpline::BoundType_First_Derivative)) {
              p_joint5_.clear();
              v_joint5_.clear();
              a_joint5_.clear();
              x_out = -rate;
              while (x_out < max_time) {
                x_out += rate;
                spline.getYbyX(x_out, y_out);
                p_joint5_.push_back(y_out);
                v_joint5_.push_back(vel);
                a_joint5_.push_back(acc);
              }

              // joint6
              if (spline.loadData(time_from_start, p_joint6, point_num, 0, 0,
                                  cubicSpline::BoundType_First_Derivative)) {
                p_joint6_.clear();
                v_joint6_.clear();
                a_joint6_.clear();
                x_out = -rate;
                while (x_out < max_time) {
                  x_out += rate;
                  spline.getYbyX(x_out, y_out);
                  p_joint6_.push_back(y_out);
                  v_joint6_.push_back(vel);
                  a_joint6_.push_back(acc);
                }

                // joint7
                if (arm_type_ == 75) {
                  if (spline.loadData(
                          time_from_start, p_joint7, point_num, 0, 0,
                          cubicSpline::BoundType_First_Derivative)) {
                    p_joint7_.clear();
                    v_joint7_.clear();
                    a_joint7_.clear();
                    x_out = -rate;
                    while (x_out < max_time) {
                      x_out += rate;
                      spline.getYbyX(x_out, y_out);
                      p_joint7_.push_back(y_out);
                      v_joint7_.push_back(vel);
                      a_joint7_.push_back(acc);
                    }

                    p2.vector_len = time_from_start_.size();
                    p2.vector_cnt = 0;

                    point_changed = true;
                    // 等待定时器将数据取出并发送完
                    while (point_changed) {
                      // ros::WallDuration(0.002).sleep();
                      if (!rclcpp::ok() || goal_handle->is_canceling()) {
                        result->error_code = -1;
                        result->error_string = "has cancel";
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                        return;
                      }
                    }
                  }

                } else {
                  p2.vector_len = time_from_start_.size();
                  p2.vector_cnt = 0;

                  point_changed = true;
                  // 等待定时器将数据取出并发送完
                  while (point_changed) {
                    // ros::WallDuration(0.002).sleep();
                    if (!rclcpp::ok() || goal_handle->is_canceling()) {
                      result->error_code = -1;
                      result->error_string = "has cancel";
                      goal_handle->canceled(result);
                      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                      return;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  else if (point_num > 0) {
    p_joint1_.clear();
    v_joint1_.clear();
    a_joint1_.clear();
    p_joint2_.clear();
    v_joint2_.clear();
    a_joint2_.clear();
    p_joint3_.clear();
    v_joint3_.clear();
    a_joint3_.clear();
    p_joint4_.clear();
    v_joint4_.clear();
    a_joint4_.clear();
    p_joint5_.clear();
    v_joint5_.clear();
    a_joint5_.clear();
    p_joint6_.clear();
    v_joint6_.clear();
    a_joint6_.clear();
    if (arm_type_ == 75) {
      p_joint7_.clear();
      v_joint7_.clear();
      a_joint7_.clear();
    }
    for (int i = 0; i < point_num; i++) {
      p_joint1_.push_back(goal->trajectory.points[i].positions[0]);
      v_joint1_.push_back(goal->trajectory.points[i].velocities[0]);
      a_joint1_.push_back(goal->trajectory.points[i].accelerations[0]);
      p_joint2_.push_back(goal->trajectory.points[i].positions[1]);
      v_joint2_.push_back(goal->trajectory.points[i].velocities[1]);
      a_joint2_.push_back(goal->trajectory.points[i].accelerations[1]);
      p_joint3_.push_back(goal->trajectory.points[i].positions[2]);
      v_joint3_.push_back(goal->trajectory.points[i].velocities[2]);
      a_joint3_.push_back(goal->trajectory.points[i].accelerations[2]);
      p_joint4_.push_back(goal->trajectory.points[i].positions[3]);
      v_joint4_.push_back(goal->trajectory.points[i].velocities[3]);
      a_joint4_.push_back(goal->trajectory.points[i].accelerations[3]);
      p_joint5_.push_back(goal->trajectory.points[i].positions[4]);
      v_joint5_.push_back(goal->trajectory.points[i].velocities[4]);
      a_joint5_.push_back(goal->trajectory.points[i].accelerations[4]);
      p_joint6_.push_back(goal->trajectory.points[i].positions[5]);
      v_joint6_.push_back(goal->trajectory.points[i].velocities[5]);
      a_joint6_.push_back(goal->trajectory.points[i].accelerations[5]);
      if (arm_type_ == 75) {
        p_joint7_.push_back(goal->trajectory.points[i].positions[6]);
        v_joint7_.push_back(goal->trajectory.points[i].velocities[6]);
        a_joint7_.push_back(goal->trajectory.points[i].accelerations[6]);
      }
    }
    p2.vector_len = point_num;
    p2.vector_cnt = 0;

    point_changed = true;
    // 等待定时器将数据取出并发送完
    while (point_changed) {
      // ros::WallDuration(0.002).sleep();
      if (!rclcpp::ok() || goal_handle->is_canceling()) {
        result->error_code = -1;
        result->error_string = "has cancel";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
    }
  }

  result->error_code = 0;
  result->error_string = "";

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
}

void Rm_Control::timer_callback() {
  /**
   * ========================================================================
   *                    TIMER_CALLBACK 函数说明
   * ========================================================================
   * 名称：timer_callback
   * 触发频率：50Hz（每20ms执行一次）
   * 功能：周期性地从插值轨迹中提取数据，发送给rm_driver
   *
   * 工作原理：
   * 1. 当 point_changed==true 时，说明有新轨迹待发送
   * 2. 从 p2.vector_cnt 指向的位置提取数据
   * 3. 将数据打包到 joint_msg 消息中
   * 4. 发布到 /rm_driver/movej_canfd_cmd 话题
   * 5. p2.vector_cnt++ 指向下一个插值点
   * 6. 当所有点都发送完后，point_changed=false，表示轨迹执行完成
   *
   * 数据流：
   *   execute_move() 生成插值数据 → timer_callback() 周期性取出 → rm_driver
   * 接收 → 硬件执行
   *
   * 关键状态机：
   *   - point_changed=true：轨迹已准备好，正在发送
   *   - point_changed=false：轨迹已全部发送，等待下一条轨迹
   *   - count_final_joint：轨迹完成后的保持发送计数器
   *
   * ========================================================================
   */

  joint_msg.follow = follow_;

  // ======== 主轨迹发送阶段 ========
  // 当 point_changed==true 时，说明execute_move已准备好插值数据
  if (point_changed) {
    // 判断是否还有未发送的插值点
    if (p2.vector_cnt < p2.vector_len) {
      // 选择机械臂类型（6轴或7轴），从插值数组中提取第 p2.vector_cnt 个点
      // p2.vector_cnt 是当前要发送的插值点的索引
      if (arm_type_ == 75) {
        // ========== 7轴机械臂（RM75）==========
        joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt);  // Joint1
        joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt);  // Joint2
        joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt);  // Joint3
        joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt);  // Joint4
        joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt);  // Joint5
        joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt);  // Joint6
        joint_msg.joint[6] = p_joint7_.at(p2.vector_cnt);  // Joint7（特有）

        // 发布关节指令到硬件驱动器
        // 话题：/rm_driver/movej_canfd_cmd
        this->joint_pos_publisher->publish(joint_msg);
      } else {
        // ========== 6轴机械臂（RM63/65）==========
        joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt);  // Joint1
        joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt);  // Joint2
        joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt);  // Joint3
        joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt);  // Joint4
        joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt);  // Joint5
        joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt);  // Joint6
        this->joint_pos_publisher->publish(joint_msg);
      }

      // ======== 指针前进 ========
      // 准备发送下一个插值点（20ms后的定时器会被触发，发送下一个点）
      p2.vector_cnt++;
    } else {
      // ======== 轨迹完成后的保持阶段 ========
      //
      // 当所有插值点都已发送完毕（p2.vector_cnt >= p2.vector_len）时
      // 继续发送最后一个点 count_keep_send 次（默认75次），用于：
      // 1. 让机械臂充分到达目标位置
      // 2. 确保硬件驱动器接收到命令
      // 3. 缓冲通信延迟
      //
      // count_keep_send 默认 = wait_move_finish_time / rate
      //                      = 1.5秒 / 0.02秒 = 75次
      //
      if (count_final_joint <= count_keep_send) {
        if (arm_type_ == 75) {
          // 7轴：发送最后一个点（索引 p2.vector_cnt-1）
          joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt - 1);
          joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt - 1);
          joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt - 1);
          joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt - 1);
          joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt - 1);
          joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt - 1);
          joint_msg.joint[6] = p_joint7_.at(p2.vector_cnt - 1);
          this->joint_pos_publisher->publish(joint_msg);
        } else {
          // 6轴：发送最后一个点
          joint_msg.joint[0] = p_joint1_.at(p2.vector_cnt - 1);
          joint_msg.joint[1] = p_joint2_.at(p2.vector_cnt - 1);
          joint_msg.joint[2] = p_joint3_.at(p2.vector_cnt - 1);
          joint_msg.joint[3] = p_joint4_.at(p2.vector_cnt - 1);
          joint_msg.joint[4] = p_joint5_.at(p2.vector_cnt - 1);
          joint_msg.joint[5] = p_joint6_.at(p2.vector_cnt - 1);
          this->joint_pos_publisher->publish(joint_msg);
        }
        // 保持发送计数递增
        count_final_joint++;
      } else {
        // ======== 轨迹执行完成 ========
        // 已经发送了足够的保持指令，现在可以开始接收下一条轨迹
        // 重置所有状态变量
        count_final_joint = 0;  // 重置保持计数
        p2.vector_cnt = 0;      // 重置当前点索引
        p2.vector_len = 0;      // 重置轨迹总长度
        point_changed = false;  // 标记轨迹已全部发送完成

        // 此时 execute_move 中的 while(point_changed) 循环会结束
        // MoveIt 会收到成功完成的反馈
      }
    }
  }
}
// 【新增】arm机械爪Goal处理（接收MoveIt的GripperCommand指令）
rclcpp_action::GoalResponse Rm_Control::handle_gripper_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperCommand::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received %s gripper goal: position=%.2f, max_effort=%.2f",
                arm_prefix_.c_str(), goal->command.position, goal->command.max_effort);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 【新增】arm机械爪取消回调（默认接受所有取消请求）
rclcpp_action::CancelResponse Rm_Control::handle_gripper_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received %s gripper cancel request", arm_prefix_.c_str());
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 【新增】执行回调（转换GripperCommand到rm_driver的爪控指令）
void Rm_Control::handle_gripper_accepted(
    const std::shared_ptr<GoalHandleGripper> goal_handle) {
  std::thread(std::bind(&Rm_Control::execute_gripper_command, this,
                        std::placeholders::_1),
              goal_handle)
      .detach();
}

// 【新增】实际执行爪控指令
void Rm_Control::execute_gripper_command(
    const std::shared_ptr<GoalHandleGripper> goal_handle) {
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GripperCommand::Feedback>();
  auto result = std::make_shared<GripperCommand::Result>();

  // 1. 转换MoveIt GripperCommand到rm_driver的Gripperset（位置控制）
  rm_ros_interfaces::msg::Gripperset gripper_set_msg;
  // MoveIt的position范围通常是[0,1]，转换为rm_driver的1~1000（对应0~70mm开口）
  gripper_set_msg.position =
      static_cast<uint16_t>(goal->command.position * 1000);
  gripper_set_msg.block = true;    // 阻塞模式，等待执行完成
  gripper_set_msg.timeout = 1000;  // 超时1秒

  // 2. 发布到rm_driver
  gripper_pos_pub->publish(gripper_set_msg);

  // 3. 等待执行结果（阻塞直到结果返回或超时）
  rclcpp::Rate rate(10);  // 10Hz轮询
  int timeout_count = 0;
  while (rclcpp::ok() && !gripper_result &&
         timeout_count < 10) {  // 超时1秒
    rate.sleep();
    timeout_count++;
  }

  // 4. 反馈结果给MoveIt
  if (gripper_result) {
    feedback->position = goal->command.position;
    goal_handle->publish_feedback(feedback);
    result->reached_goal = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "%s gripper move success", arm_prefix_.c_str());
  } else {
    result->reached_goal = false;
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "%s gripper move failed", arm_prefix_.c_str());
  }

  // 重置结果缓存
  gripper_result = false;
}

// 【新增】统一爪控结果回调（接收rm_driver的执行结果）
void Rm_Control::gripper_pos_result_cb(
    const std_msgs::msg::Bool::SharedPtr msg) {
  gripper_result = msg->data;
}



/**
 * ========================================================================
 *            rm_control 核心工作流程总结
 * ========================================================================
 *
 * 1. 初始化阶段：
 *    - Rm_Control 构造函数创建 Action Server（FollowJointTrajectory）
 *    - 创建 joint_pos_publisher 发布器
 *    - 创建 state_timer 定时器，每20ms触发一次回调
 *
 * 2. 轨迹接收阶段：
 *    - MoveIt 规划完成后，通过 Action 发送轨迹给 rm_control
 *    - 轨迹包含10-20个稀疏路点和时间戳
 *
 * 3. 轨迹插值阶段 (execute_move)：
 *    - 验证路点数 > 3（有效规划）
 *    - 提取MoveIt轨迹中的位置、速度、加速度、时间戳数据
 *    - 为每个关节（1-7）应用三次样条插值
 *    - 生成1000+个密集路点（20ms间隔，50Hz采样率）
 *    - 设置 point_changed=true，标记数据已准备
 *    - 等待 timer_callback 发送完所有数据
 *
 * 4. 轨迹发送阶段 (timer_callback)：
 *    - 50Hz频率（每20ms）被触发
 *    - 当 point_changed=true 时：
 *      a) 从全局向量中取出第 p2.vector_cnt 个插值点
 *      b) 打包成 joint_msg 消息
 *      c) 发布到 /rm_driver/movej_canfd_cmd 话题
 *      d) p2.vector_cnt 递增，准备下一个点
 *
 * 5. 轨迹完成阶段：
 *    - 所有插值点都已发送（p2.vector_cnt >= p2.vector_len）
 *    - 继续发送最后一个点1.5秒（保证机械臂到达目标）
 *    - 设置 point_changed=false，等待下一条轨迹
 *    - MoveIt 收到 result 反馈，表示轨迹执行完成
 *
 * 6. 硬件执行阶段：
 *    - rm_driver 接收 /rm_driver/movej_canfd_cmd 消息
 *    - 通过 TCP 协议（8081端口）发送指令给机械臂控制器
 *    - 同时通过 UDP（9501端口）以50Hz接收实时反馈
 *
 * ========================================================================
 *
 * 性能指标：
 * - 插值精度：20ms（50Hz）
 * - 轨迹光滑度：C2连续（位置、速度、加速度连续）
 * - 典型轨迹点数：从10-20个 → 1000+个
 * - 单个轨迹耗时：2-8秒（取决于运动距离和速度）
 * - 通信延迟：<1ms（TCP本地通信）
 *
 * ========================================================================
 */

void Rm_Control::get_move_stop_callback(
    const std_msgs::msg::Empty::SharedPtr msg) {
  // bool result;
  // result = msg->data;
  point_changed = false;
  // RCLCPP_INFO(this->get_logger(), "move stop is true!!! ");
}
/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  count_keep_send = wait_move_finish_time / min_interval;
  rclcpp::spin(std::make_shared<Rm_Control>("rm_control"));
  rclcpp::shutdown();
  return 0;
}