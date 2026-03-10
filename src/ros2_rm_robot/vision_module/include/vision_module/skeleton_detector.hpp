#ifndef VISION_MODULE_SKELETON_DETECTOR_HPP
#define VISION_MODULE_SKELETON_DETECTOR_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_interfaces/msg/skeleton.hpp"
#include "vision_interfaces/msg/skeleton_joint.hpp"

namespace vision_module {

/**
 * 人体骨骼检测节点 (Skeleton Detection)
 *
 * 功能：
 * - 从 Kinect 或 RGB-D 相机接收深度图和 RGB 图
 * - 检测人体骨骼关键点
 * - 发布骨骼关节位置
 *
 * 关键点定义 (17 个)：
 * 0-鼻子, 1-左眼, 2-右眼, 3-左耳, 4-右耳
 * 5-左肩, 6-右肩, 7-左肘, 8-右肘, 9-左腕, 10-右腕
 * 11-左髋, 12-右髋, 13-左膝, 14-右膝, 15-左踝, 16-右踝
 */
class SkeletonDetectorNode : public rclcpp::Node {
 public:
  SkeletonDetectorNode();
  ~SkeletonDetectorNode() = default;

  // 骨骼关键点的枚举
  enum SkeletonKeypoint {
    NOSE = 0,
    LEFT_EYE = 1,
    RIGHT_EYE = 2,
    LEFT_EAR = 3,
    RIGHT_EAR = 4,
    LEFT_SHOULDER = 5,
    RIGHT_SHOULDER = 6,
    LEFT_ELBOW = 7,
    RIGHT_ELBOW = 8,
    LEFT_WRIST = 9,
    RIGHT_WRIST = 10,
    LEFT_HIP = 11,
    RIGHT_HIP = 12,
    LEFT_KNEE = 13,
    RIGHT_KNEE = 14,
    LEFT_ANKLE = 15,
    RIGHT_ANKLE = 16,
    NUM_KEYPOINTS = 17
  };

 private:
  // 参数
  std::string detection_model_;
  double confidence_threshold_;
  bool use_depth_;

  // 状态
  std::vector<cv::Point2f> last_skeleton_;
  float person_distance_ = 0.0f;  // 人距摄像机的距离

  // 订阅器和发布器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<vision_interfaces::msg::Skeleton>::SharedPtr skeleton_pub_;

  // 回调函数
  void rgb_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // 骨骼检测
  std::vector<cv::Point2f> detect_skeleton(const cv::Mat& image);
  void draw_skeleton(cv::Mat& image, const std::vector<cv::Point2f>& keypoints);
  float estimate_depth(const cv::Mat& depth_image, const cv::Point2f& keypoint);
};

}  // namespace vision_module

#endif
