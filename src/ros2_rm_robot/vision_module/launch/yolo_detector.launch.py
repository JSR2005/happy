#!/usr/bin/env python3
"""
YOLOv8 检测器启动脚本

使用方法:
  ros2 launch vision_module yolo_detector.launch.py
  
参数:
  model_path:=<path>  指定模型路径
  use_gpu:=true       启用 GPU 加速
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取 vision_module 包的路径
    vision_module_dir = get_package_share_directory('vision_module')
    config_file = os.path.join(vision_module_dir, 'config', 'yolo_detector_params.yaml')
    
    # 声明启动参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/jsr/models/yolov8m.onnx',
        description='Path to YOLOv8 ONNX model'
    )
    
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='false',
        description='Whether to use GPU acceleration'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/',
        description='ROS namespace'
    )
    
    # 创建节点
    yolo_detector_node = Node(
        package='vision_module',
        executable='yolo_detector_node',
        name='yolo_detector',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            config_file,
            {
                'model_path': LaunchConfiguration('model_path'),
                'use_gpu': LaunchConfiguration('use_gpu'),
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
    )
    
    # 日志信息
    log_msg = LogInfo(
        msg=[
            'YOLOv8 Detector Node launched with model: ',
            LaunchConfiguration('model_path'),
            ' | GPU: ',
            LaunchConfiguration('use_gpu'),
        ]
    )
    
    return LaunchDescription([
        model_path_arg,
        use_gpu_arg,
        namespace_arg,
        log_msg,
        yolo_detector_node,
    ])
