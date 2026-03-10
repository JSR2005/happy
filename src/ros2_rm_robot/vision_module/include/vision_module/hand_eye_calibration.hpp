#ifndef VISION_MODULE_HAND_EYE_CALIBRATION_HPP
#define VISION_MODULE_HAND_EYE_CALIBRATION_HPP

#include <Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace vision_module {

/**
 * 手眼标定节点 (Eye-in-Hand Calibration)
 *
 * 功能：
 * - 接收机械臂末端位姿数据
 * - 识别标定板（棋盘格）
 * - 计算摄像机与末端执行器的相对变换
 * - 发布标定结果
 *
 * 原理：
 * 通过多个位置的标定板和对应的末端位姿，求解摄像机坐标系到末端坐标系的变换
 * T_camera_to_end = T_base_to_camera^(-1) * T_base_to_end
 */
class HandEyeCalibrationNode : public rclcpp::Node {
 public:
  explicit HandEyeCalibrationNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~HandEyeCalibrationNode() = default;

 private:
  // 标定数据点结构
  struct CalibrationPoint {
    cv::Mat R_board_to_camera;    // 标定板到摄像机的旋转
    cv::Vec3d t_board_to_camera;  // 标定板到摄像机的平移
    cv::Mat R_base_to_end;        // 基座到末端的旋转
    cv::Vec3d t_base_to_end;      // 基座到末端的平移
  };

  // 参数
  int chessboard_width_;
  int chessboard_height_;
  float square_size_;
  int num_calibration_points_;

  // 状态
  std::vector<CalibrationPoint> calibration_points_;
  bool calibration_done_ = false;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // 订阅器和发布器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr
      calib_result_pub_;

  // 回调函数
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // 标定处理
  bool detect_chessboard(const cv::Mat& image,
                         std::vector<cv::Point2f>& corners);
  void add_calibration_point(const cv::Mat& image,
                             const geometry_msgs::msg::PoseStamped& pose);
  bool compute_hand_eye_calibration();

  // 工具函数
  cv::Mat pose_to_matrix(const geometry_msgs::msg::Pose& pose);
  geometry_msgs::msg::Transform matrix_to_transform(const cv::Mat& matrix);
  void load_camera_intrinsics();
};

}  // namespace vision_module

#endif
