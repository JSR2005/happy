#include "vision_module/yolo_detector.hpp"

#include <filesystem>

#include "vision_interfaces/msg/detection.hpp"

namespace vision_module {

YOLODetectorNode::YOLODetectorNode(const rclcpp::NodeOptions& options)
    : Node("yolo_detector", options) {
  RCLCPP_INFO(this->get_logger(), "初始化 YOLOv8 检测器...");

  // 声明参数
  this->declare_parameter("model_path", "/home/jsr/models/yolov8m.onnx");
  this->declare_parameter("confidence_threshold", 0.45);
  this->declare_parameter("nms_threshold", 0.5);
  this->declare_parameter("input_size", 640);
  this->declare_parameter("use_gpu", false);
  this->declare_parameter("class_names_file", "/home/jsr/models/coco.names");

  // 读取参数
  model_path_ = this->get_parameter("model_path").as_string();
  conf_threshold_ = this->get_parameter("confidence_threshold").as_double();
  nms_threshold_ = this->get_parameter("nms_threshold").as_double();
  input_size_ = this->get_parameter("input_size").as_int();
  use_gpu_ = this->get_parameter("use_gpu").as_bool();

  // 检查模型文件是否存在
  if (!std::filesystem::exists(model_path_)) {
    RCLCPP_WARN(this->get_logger(), "模型文件不存在: %s, 将使用仿真模式",
                model_path_.c_str());
    model_path_ = "";
  } else {
    // 加载 ONNX 模型
    try {
      model_ = cv::dnn::readNetFromONNX(model_path_);

      if (use_gpu_) {
        model_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        model_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        RCLCPP_INFO(this->get_logger(), "✓ 使用 GPU 加速 (CUDA)");
      } else {
        model_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        model_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        RCLCPP_INFO(this->get_logger(), "✓ 使用 CPU 推理");
      }

      RCLCPP_INFO(this->get_logger(), "✓ 模型加载成功: %s",
                  model_path_.c_str());
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "❌ 模型加载失败: %s", e.what());
      model_path_ = "";
    }
  }

  // 加载类别名称
  load_class_names();

  // 创建图像传输
  it_ = std::make_unique<image_transport::ImageTransport>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}));

  // 创建订阅器 - 从 RealSense 订阅 RGB 图像
  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->image_callback(msg);
      });

  // 创建发布器 - 发布检测结果
  detection_pub_ =
      this->create_publisher<vision_interfaces::msg::DetectionArray>(
          "/detection_results", rclcpp::SensorDataQoS());

  // 创建发布器 - 发布调试图像
  debug_pub_ = it_->advertise("/detection_debug_image", 1);

  RCLCPP_INFO(this->get_logger(),
              "✓ YOLOv8 检测器初始化完成 "
              "(置信度=%.2f, NMS=%.2f, 输入尺寸=%d)",
              conf_threshold_, nms_threshold_, input_size_);
}

void YOLODetectorNode::load_class_names() {
  // COCO 数据集的 80 个类别
  class_names_ = {"person",
                  "bicycle",
                  "car",
                  "motorcycle",
                  "airplane",
                  "bus",
                  "train",
                  "truck",
                  "boat",
                  "traffic light",
                  "fire hydrant",
                  "stop sign",
                  "parking meter",
                  "bench",
                  "cat",
                  "dog",
                  "horse",
                  "sheep",
                  "cow",
                  "elephant",
                  "bear",
                  "zebra",
                  "giraffe",
                  "backpack",
                  "umbrella",
                  "handbag",
                  "tie",
                  "suitcase",
                  "frisbee",
                  "skis",
                  "snowboard",
                  "sports ball",
                  "kite",
                  "baseball bat",
                  "baseball glove",
                  "skateboard",
                  "surfboard",
                  "tennis racket",
                  "bottle",
                  "wine glass",
                  "cup",
                  "fork",
                  "knife",
                  "spoon",
                  "bowl",
                  "banana",
                  "apple",
                  "sandwich",
                  "orange",
                  "broccoli",
                  "carrot",
                  "hot dog",
                  "pizza",
                  "donut",
                  "cake",
                  "chair",
                  "couch",
                  "potted plant",
                  "bed",
                  "dining table",
                  "toilet",
                  "tv",
                  "laptop",
                  "mouse",
                  "remote",
                  "keyboard",
                  "microwave",
                  "oven",
                  "toaster",
                  "sink",
                  "refrigerator",
                  "book",
                  "clock",
                  "vase",
                  "scissors",
                  "teddy bear",
                  "hair drier",
                  "toothbrush"};
}

void YOLODetectorNode::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // 转换 ROS 图像到 OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image = cv_ptr->image.clone();

    // 记录处理时间
    auto start_time = std::chrono::high_resolution_clock::now();

    // 运行检测
    std::vector<Detection> detections = detect(image);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    // 绘制检测框
    draw_detections(image, detections);

    // 发布检测结果
    auto detection_msg = vision_interfaces::msg::DetectionArray();
    detection_msg.header = msg->header;
    detection_msg.detections.reserve(detections.size());

    for (const auto& det : detections) {
      vision_interfaces::msg::Detection d;
      d.class_id = det.class_id;
      d.class_name = det.class_name;
      d.confidence = det.confidence;
      d.bbox_x = det.bbox.x;
      d.bbox_y = det.bbox.y;
      d.bbox_width = det.bbox.width;
      d.bbox_height = det.bbox.height;
      d.center_x = det.center.x;
      d.center_y = det.center.y;
      detection_msg.detections.push_back(d);
    }

    detection_pub_->publish(detection_msg);

    // 发布调试图像
    cv_bridge::CvImage debug_img;
    debug_img.header = msg->header;
    debug_img.encoding = "bgr8";
    debug_img.image = image;
    debug_pub_.publish(debug_img.toImageMsg());

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "检测完成: %ld 个物体, 耗时: %ld ms",
                         detections.size(), duration.count());

  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV 异常: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "异常: %s", e.what());
  }
}

std::vector<YOLODetectorNode::Detection> YOLODetectorNode::detect(
    const cv::Mat& image) {
  std::vector<Detection> detections;

  if (model_path_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "模型未加载，返回空检测结果");
    return detections;
  }

  // 预处理：创建 blob
  cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 255.0,
                                        cv::Size(input_size_, input_size_),
                                        cv::Scalar(0, 0, 0), true, false);

  // 设置输入
  model_.setInput(blob);

  // 前向传播
  std::vector<cv::Mat> outputs;
  std::vector<std::string> output_names = model_.getUnconnectedOutLayersNames();
  model_.forward(outputs, output_names);

  // 后处理
  postprocess(image, outputs, detections);

  return detections;
}

void YOLODetectorNode::postprocess(const cv::Mat& image,
                                   const std::vector<cv::Mat>& outputs,
                                   std::vector<Detection>& detections) {
  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  float scale_x = static_cast<float>(image.cols) / input_size_;
  float scale_y = static_cast<float>(image.rows) / input_size_;

  // 处理 YOLOv8 的输出格式
  for (const auto& output : outputs) {
    const float* data = reinterpret_cast<const float*>(output.data);
    int rows = output.rows;
    int cols = output.cols;

    for (int i = 0; i < rows; ++i) {
      const float* row_data = data + i * cols;

      // YOLOv8: [x, y, w, h, conf, class_0, class_1, ...]
      float x = row_data[0];
      float y = row_data[1];
      float w = row_data[2];
      float h = row_data[3];
      float obj_conf = row_data[4];

      // 找到最高的类别置信度
      float max_conf = 0.0f;
      int max_class = -1;

      int num_classes = cols - 5;
      for (int j = 0; j < num_classes; ++j) {
        float class_conf = row_data[5 + j];
        if (class_conf > max_conf) {
          max_conf = class_conf;
          max_class = j;
        }
      }

      float final_conf = obj_conf * max_conf;

      // 过滤低置信度检测
      if (final_conf > conf_threshold_ && max_class >= 0) {
        // 将坐标从 [0, 1] 缩放到图像大小
        int x_min = std::max(0, static_cast<int>((x - w / 2) * scale_x));
        int y_min = std::max(0, static_cast<int>((y - h / 2) * scale_y));
        int width = std::min(image.cols - x_min, static_cast<int>(w * scale_x));
        int height =
            std::min(image.rows - y_min, static_cast<int>(h * scale_y));

        classIds.push_back(max_class);
        confidences.push_back(final_conf);
        boxes.push_back(cv::Rect(x_min, y_min, width, height));
      }
    }
  }

  // NMS (非最大值抑制)
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_,
                    indices);

  // 收集最终检测结果
  for (int idx : indices) {
    Detection det;
    det.class_id = classIds[idx];
    det.confidence = confidences[idx];
    det.bbox = boxes[idx];
    det.center.x = det.bbox.x + det.bbox.width / 2.0f;
    det.center.y = det.bbox.y + det.bbox.height / 2.0f;

    if (det.class_id < static_cast<int>(class_names_.size())) {
      det.class_name = class_names_[det.class_id];
    } else {
      det.class_name = "unknown";
    }

    detections.push_back(det);
  }
}

void YOLODetectorNode::draw_detections(
    cv::Mat& image, const std::vector<Detection>& detections) {
  for (const auto& det : detections) {
    // 绘制边界框
    cv::rectangle(image, det.bbox, cv::Scalar(0, 255, 0), 2);

    // 准备标签文本
    std::string label =
        cv::format("%s: %.2f", det.class_name.c_str(), det.confidence);

    // 获取文本大小
    int baseline = 0;
    cv::Size text_size =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);

    // 绘制背景矩形
    cv::rectangle(image,
                  cv::Point(det.bbox.x, det.bbox.y - text_size.height - 10),
                  cv::Point(det.bbox.x + text_size.width, det.bbox.y),
                  cv::Scalar(0, 255, 0), cv::FILLED);

    // 绘制文本
    cv::putText(image, label, cv::Point(det.bbox.x, det.bbox.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 绘制中心点
    cv::circle(image,
               cv::Point(static_cast<int>(det.center.x),
                         static_cast<int>(det.center.y)),
               5, cv::Scalar(0, 0, 255), -1);
  }
}

}  // namespace vision_module

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision_module::YOLODetectorNode)
