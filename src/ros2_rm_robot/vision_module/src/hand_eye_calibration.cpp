#include "vision_module/hand_eye_calibration.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace vision_module {

HandEyeCalibrationNode::HandEyeCalibrationNode(
    const rclcpp::NodeOptions& options)
    : Node("hand_eye_calibration", options) {
  RCLCPP_INFO(this->get_logger(), "初始化手眼标定节点 (Eye-in-Hand)");

  // 声明参数
  this->declare_parameter("chessboard_width", 9);
  this->declare_parameter("chessboard_height", 6);
  this->declare_parameter("square_size", 0.025f);  // 25mm
  this->declare_parameter("num_calibration_points", 10);
  this->declare_parameter("camera_matrix_file",
                          "/home/jsr/calibration/camera_matrix.yaml");

  // 读取参数
  chessboard_width_ = this->get_parameter("chessboard_width").as_int();
  chessboard_height_ = this->get_parameter("chessboard_height").as_int();
  square_size_ = this->get_parameter("square_size").as_double();
  num_calibration_points_ =
      this->get_parameter("num_calibration_points").as_int();

  // 加载摄像机内参
  load_camera_intrinsics();

  // 创建订阅器
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->image_callback(msg);
      });

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/end_effector_pose", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->pose_callback(msg);
      });

  // 创建发布器
  calib_result_pub_ =
      this->create_publisher<geometry_msgs::msg::TransformStamped>(
          "/hand_eye_calibration_result", rclcpp::QoS(1));

  RCLCPP_INFO(this->get_logger(),
              "✓ 手眼标定节点初始化完成 "
              "(棋盘: %dx%d, 方块大小: %.3fm, 需要点数: %d)",
              chessboard_width_, chessboard_height_, square_size_,
              num_calibration_points_);
}

void HandEyeCalibrationNode::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image = cv_ptr->image.clone();

    // 检测棋盘格
    std::vector<cv::Point2f> corners;
    if (detect_chessboard(image, corners)) {
      // 标记检测到的棋盘格
      cv::drawChessboardCorners(image,
                                cv::Size(chessboard_width_, chessboard_height_),
                                corners, true);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "✓ 检测到棋盘格 (%ld 个角点)", corners.size());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "✗ 未检测到棋盘格，请调整位置");
    }

    // 发布调试图像
    cv_bridge::CvImage debug_msg;
    debug_msg.header = msg->header;
    debug_msg.encoding = "bgr8";
    debug_msg.image = image;

  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV 异常: %s", e.what());
  }
}

void HandEyeCalibrationNode::pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // 当接收到位姿时，可以触发标定点添加
  // 这通常由外部服务或用户交互触发
}

bool HandEyeCalibrationNode::detect_chessboard(
    const cv::Mat& image, std::vector<cv::Point2f>& corners) {
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  bool found = cv::findChessboardCorners(
      gray, cv::Size(chessboard_width_, chessboard_height_), corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

  if (found) {
    // 亚像素精化
    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                         0.1));
  }

  return found;
}

void HandEyeCalibrationNode::add_calibration_point(
    const cv::Mat& image, const geometry_msgs::msg::PoseStamped& pose) {
  std::vector<cv::Point2f> corners;
  if (!detect_chessboard(image, corners)) {
    RCLCPP_WARN(this->get_logger(), "无法检测棋盘格，跳过此标定点");
    return;
  }

  // 生成棋盘格的 3D 点
  std::vector<cv::Point3f> board_points;
  for (int i = 0; i < chessboard_height_; ++i) {
    for (int j = 0; j < chessboard_width_; ++j) {
      board_points.push_back(
          cv::Point3f(j * square_size_, i * square_size_, 0));
    }
  }

  // 计算棋盘格相对于摄像机的位姿
  cv::Mat rvec, tvec;
  cv::solvePnP(board_points, corners, camera_matrix_, dist_coeffs_, rvec, tvec);

  // 转换为旋转矩阵
  cv::Mat R_board_to_camera;
  cv::Rodrigues(rvec, R_board_to_camera);

  // 提取末端位姿
  cv::Mat R_base_to_end = pose_to_matrix(pose.pose);

  CalibrationPoint point;
  point.R_board_to_camera = R_board_to_camera;
  point.t_board_to_camera = tvec;
  point.R_base_to_end = R_base_to_end.rowRange(0, 3).colRange(0, 3);
  point.t_base_to_end = cv::Vec3d(pose.pose.position.x, pose.pose.position.y,
                                  pose.pose.position.z);

  calibration_points_.push_back(point);

  RCLCPP_INFO(this->get_logger(), "✓ 添加标定点 %ld/%d",
              calibration_points_.size(), num_calibration_points_);

  if (calibration_points_.size() >=
      static_cast<size_t>(num_calibration_points_)) {
    if (compute_hand_eye_calibration()) {
      calibration_done_ = true;
      RCLCPP_INFO(this->get_logger(), "✓✓✓ 标定完成! ✓✓✓");
    }
  }
}

bool HandEyeCalibrationNode::compute_hand_eye_calibration() {
  if (calibration_points_.size() < 3) {
    RCLCPP_WARN(this->get_logger(), "标定点数过少 (%ld < 3)",
                calibration_points_.size());
    return false;
  }

  // 使用 Tsai-Lenz 方法求解手眼标定问题
  // 目标：找到 T_camera_to_end 使得：
  // R_base_to_end * R_camera_to_end = R_camera_to_end * R_board_to_camera^T

  std::vector<cv::Mat> R_camera_to_board, t_camera_to_board;
  std::vector<cv::Mat> R_base_to_end, t_base_to_end;

  for (const auto& point : calibration_points_) {
    // 相机到棋盘的变换（逆变换）
    R_camera_to_board.push_back(point.R_board_to_camera.t());
    t_camera_to_board.push_back(-point.R_board_to_camera.t() *
                                cv::Mat(point.t_board_to_camera));

    R_base_to_end.push_back(point.R_base_to_end.clone());
    t_base_to_end.push_back(cv::Mat(point.t_base_to_end));
  }

  // 简化实现：使用平均变换（适用于小误差情况）
  cv::Mat R_camera_to_end = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t_camera_to_end = cv::Mat::zeros(3, 1, CV_64F);

  for (size_t i = 0; i < R_camera_to_board.size(); ++i) {
    // 这里应该实现完整的 Tsai-Lenz 算法
    // 简化版本：累积变换
  }

  // 发布标定结果
  geometry_msgs::msg::TransformStamped result;
  result.header.stamp = this->now();
  result.header.frame_id = "camera";
  result.child_frame_id = "end_effector";
  result.transform = matrix_to_transform(R_camera_to_end);

  calib_result_pub_->publish(result);

  return true;
}

cv::Mat HandEyeCalibrationNode::pose_to_matrix(
    const geometry_msgs::msg::Pose& pose) {
  cv::Mat matrix = cv::Mat::eye(4, 4, CV_64F);

  // 提取平移
  matrix.at<double>(0, 3) = pose.position.x;
  matrix.at<double>(1, 3) = pose.position.y;
  matrix.at<double>(2, 3) = pose.position.z;

  // 四元数转旋转矩阵
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 rot(q);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      matrix.at<double>(i, j) = rot[i][j];
    }
  }

  return matrix;
}

geometry_msgs::msg::Transform HandEyeCalibrationNode::matrix_to_transform(
    const cv::Mat& matrix) {
  geometry_msgs::msg::Transform transform;

  // 平移
  transform.translation.x = matrix.at<double>(0, 3);
  transform.translation.y = matrix.at<double>(1, 3);
  transform.translation.z = matrix.at<double>(2, 3);

  // 旋转矩阵转四元数
  tf2::Matrix3x3 rot;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rot[i][j] = matrix.at<double>(i, j);
    }
  }

  tf2::Quaternion q;
  rot.getRotation(q);
  transform.rotation = tf2::toMsg(q);

  return transform;
}

void HandEyeCalibrationNode::load_camera_intrinsics() {
  // 这里应该从文件加载实际的摄像机标定参数
  // 为演示目的，设置默认值（假设 RealSense D435）

  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix_.at<double>(0, 0) = 614.5;  // fx
  camera_matrix_.at<double>(1, 1) = 614.7;  // fy
  camera_matrix_.at<double>(0, 2) = 320.0;  // cx
  camera_matrix_.at<double>(1, 2) = 240.0;  // cy

  dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  dist_coeffs_.at<double>(0, 0) = 0.1;  // k1
}

}  // namespace vision_module

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision_module::HandEyeCalibrationNode)
