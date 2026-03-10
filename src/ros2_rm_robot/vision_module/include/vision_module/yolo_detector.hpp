#ifndef VISION_MODULE_YOLO_DETECTOR_HPP
#define VISION_MODULE_YOLO_DETECTOR_HPP

#include <memory>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_interfaces/msg/detection_array.hpp"

namespace vision_module {

/**
 * YOLOv8 目标检测节点
 * 功能：
 * - 订阅 RealSense RGB 图像
 * - 运行 YOLOv8 目标检测
 * - 发布检测结果和可视化图像
 */
class YOLODetectorNode : public rclcpp::Node {
 public:
  explicit YOLODetectorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~YOLODetectorNode() = default;

 private:
  // 检测结果结构
  struct Detection {
    int class_id;
    float confidence;
    cv::Rect bbox;
    cv::Point2f center;
    std::string class_name;
  };

  // 参数
  std::string model_path_;
  double conf_threshold_;
  double nms_threshold_;
  int input_size_;
  bool use_gpu_;
  std::vector<std::string> class_names_;

  // 模型
  cv::dnn::Net model_;

  // 订阅器和发布器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Publisher<vision_interfaces::msg::DetectionArray>::SharedPtr
      detection_pub_;
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher debug_pub_;

  // 回调函数
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // 检测处理
  std::vector<Detection> detect(const cv::Mat& image);
  void postprocess(const cv::Mat& image, const std::vector<cv::Mat>& outputs,
                   std::vector<Detection>& detections);
  void draw_detections(cv::Mat& image,
                       const std::vector<Detection>& detections);

  // 工具函数
  void load_class_names();
};

}  // namespace vision_module

#endif  // VISION_MODULE_YOLO_DETECTOR_HPP
