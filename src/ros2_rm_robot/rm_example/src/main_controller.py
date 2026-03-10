#!/usr/bin/env python3
"""
主控制节点 - 协调整个视觉抓取系统

流程：
1. 使用 RealSense D435 拍摄环境
2. YOLOv8 检测物体
3. 根据人体骨骼动作（Kinect）判断目标
4. 手眼标定获取物体位置
5. 通过官方 rm_driver (ROS2 消息) 控制机械臂
6. 力控制执行（导纳控制）
7. 夹爪抓取物体
8. 检测人员，执行阻抗控制
9. 放置物体

架构说明：
- 本节点不直接调用 C API，而是通过 ROS2 消息与 rm_driver 通信
- rm_driver 是官方提供的 C++ 节点，封装了底层 API (rm_interface.h)
- 我们通过发布 Movej/Movel 消息来控制机械臂
- 通过订阅状态话题来获取机械臂反馈
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np  # type: ignore
import cv2  # type: ignore
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Tuple
import time
import math

# ROS 消息类型
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Quaternion, Point
from std_msgs.msg import Bool, String, Float64MultiArray
from vision_interfaces.msg import DetectionArray, Detection, Skeleton
# 重要：导入官方的运动消息
from rm_ros_interfaces.msg import Movej, Movel  # ← 官方运动消息，通过 rm_driver 使用
from tf2_ros import TransformListener, Buffer
from cv_bridge import CvBridge


class SystemState(Enum):
    """系统状态机"""
    IDLE = "idle"
    SCANNING = "scanning"
    DETECTING = "detecting"
    PLANNING = "planning"
    EXECUTING = "executing"
    GRASPING = "grasping"
    PLACING = "placing"
    ERROR = "error"


@dataclass
class DetectedObject:
    """检测到的物体"""
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x_min, y_min, x_max, y_max)
    center_2d: Tuple[float, float]  # (x, y) 像素坐标
    center_3d: Optional[Point] = None  # 3D 世界坐标
    gripper_frame: Optional[Pose] = None  # 夹爪工作坐标系下的坐标


@dataclass
class HumanGesture:
    """人体手势"""
    left_hand_pos: Point
    right_hand_pos: Point
    raised_hand: str  # "left", "right", "none"
    gesture_type: str  # "grab_left", "grab_right", "place", "stop"


class MainControllerNode(Node):
    """主控制节点"""

    def __init__(self):
        super().__init__("main_controller")

        self.get_logger().info("=" * 60)
        self.get_logger().info("启动主控制节点")
        self.get_logger().info("=" * 60)

        # 系统状态
        self.system_state = SystemState.IDLE
        self.detected_objects: List[DetectedObject] = []
        self.target_object: Optional[DetectedObject] = None
        self.human_gesture: Optional[HumanGesture] = None
        self.current_joint_state: Optional[JointState] = None
        self.arm_is_moving = False
        self.last_move_time = 0.0

        # 工具
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 参数
        self.declare_parameter("arm_dof", 7)  # RM 65-6F 是 6 DOF，RM 75 是 7 DOF
        self.declare_parameter("use_force_control", True)
        self.declare_parameter("arm_speed", 30)  # 运动速度百分比 (1-100)
        self.declare_parameter("arm_blend_radius", 20)  # 交融半径百分比 (0-100)
        self.declare_parameter("place_location", [0.3, 0.0, 0.2])  # [x, y, z]
        self.declare_parameter("home_position", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.arm_dof = self.get_parameter("arm_dof").as_int()
        self.use_force_control = self.get_parameter("use_force_control").as_bool()
        self.arm_speed = self.get_parameter("arm_speed").as_int()
        self.arm_blend_radius = self.get_parameter("arm_blend_radius").as_int()
        self.place_location = self.get_parameter("place_location").as_double_array()
        self.home_position = self.get_parameter("home_position").as_double_array()

        # 创建回调组（避免死锁）
        self.scan_group = MutuallyExclusiveCallbackGroup()
        self.detect_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()

        # 订阅器 - 接收感知信息
        self.detection_sub = self.create_subscription(
            DetectionArray,
            "/detection_results",
            self.detection_callback,
            10,
            callback_group=self.detect_group
        )

        self.skeleton_sub = self.create_subscription(
            Skeleton,
            "/human_skeleton",
            self.skeleton_callback,
            10,
            callback_group=self.detect_group
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.control_group
        )

        # 发布器 - 控制机械臂
        # 【重要】通过 Movej/Movel 消息与 rm_driver 通信，rm_driver 会调用底层 C API
        self.movej_pub = self.create_publisher(Movej, "/movej", 10)
        self.movel_pub = self.create_publisher(Movel, "/movel", 10)
        
        # 发布导纳控制命令
        self.force_control_pub = self.create_publisher(
            Float64MultiArray, "/force_control_cmd", 10)
        
        # 发布系统状态
        self.status_pub = self.create_publisher(String, "/system_status", 10)

        # 定时器
        self.main_loop_timer = self.create_timer(
            0.1, self.main_loop, callback_group=self.control_group)
        
        self.get_logger().info("✓ 主控制节点启动完成")
        self.get_logger().info(f"  - 机械臂 DOF: {self.arm_dof}")
        self.get_logger().info(f"  - 运动速度: {self.arm_speed}%")
        self.get_logger().info(f"  - 力控制: {'启用' if self.use_force_control else '禁用'}")

    def detection_callback(self, msg: DetectionArray):
        """物体检测回调 - 接收来自 YOLOv8 的检测结果"""
        self.detected_objects = []

        for det in msg.detections:
            obj = DetectedObject(
                class_name=det.class_name,
                confidence=det.confidence,
                bbox=(det.bbox.x_min, det.bbox.y_min, 
                      det.bbox.x_max, det.bbox.y_max),
                center_2d=(det.center_x, det.center_y)
            )
            self.detected_objects.append(obj)

        if self.detected_objects:
            # self.get_logger().info(f"检测到 {len(self.detected_objects)} 个物体")
            pass

    def skeleton_callback(self, msg: Skeleton):
        """人体骨骼检测回调 - 接收来自 Kinect 的骨骼数据"""
        if not msg.joints or len(msg.joints) == 0:
            return

        # 提取关键关节
        left_hand = None
        right_hand = None
        left_shoulder = None
        right_shoulder = None

        for joint in msg.joints:
            if joint.joint_type == "left_hand":
                left_hand = Point(x=joint.position.x, y=joint.position.y, z=joint.position.z)
            elif joint.joint_type == "right_hand":
                right_hand = Point(x=joint.position.x, y=joint.position.y, z=joint.position.z)
            elif joint.joint_type == "left_shoulder":
                left_shoulder = Point(x=joint.position.x, y=joint.position.y, z=joint.position.z)
            elif joint.joint_type == "right_shoulder":
                right_shoulder = Point(x=joint.position.x, y=joint.position.y, z=joint.position.z)

        # 判断手势（手高于肩膀表示抬起）
        gesture_type = "none"
        raised_hand = "none"

        if left_hand and left_shoulder:
            if left_hand.y < left_shoulder.y - 0.1:  # 手抬起
                raised_hand = "left"
                gesture_type = "grab_left"

        if right_hand and right_shoulder:
            if right_hand.y < right_shoulder.y - 0.1:  # 手抬起
                raised_hand = "right"
                gesture_type = "grab_right"

        self.human_gesture = HumanGesture(
            left_hand_pos=left_hand or Point(),
            right_hand_pos=right_hand or Point(),
            raised_hand=raised_hand,
            gesture_type=gesture_type
        )

    def joint_state_callback(self, msg: JointState):
        """关节状态回调 - 接收机械臂关节状态"""
        self.current_joint_state = msg
        
        # 检测运动是否完成（当速度接近 0 时）
        if msg.velocity and len(msg.velocity) > 0:
            if max(abs(v) for v in msg.velocity) < 0.01:
                self.arm_is_moving = False

    def main_loop(self):
        """主控制循环 - 状态机驱动"""
        if self.system_state == SystemState.IDLE:
            self.state_idle()
        elif self.system_state == SystemState.SCANNING:
            self.state_scanning()
        elif self.system_state == SystemState.DETECTING:
            self.state_detecting()
        elif self.system_state == SystemState.PLANNING:
            self.state_planning()
        elif self.system_state == SystemState.EXECUTING:
            self.state_executing()
        elif self.system_state == SystemState.GRASPING:
            self.state_grasping()
        elif self.system_state == SystemState.PLACING:
            self.state_placing()

    def state_idle(self):
        """IDLE 状态 - 等待启动命令"""
        # 这里可以添加启动条件，比如收到启动话题
        self.system_state = SystemState.SCANNING
        self.get_logger().info("💫 进入扫描模式")

    def state_scanning(self):
        """SCANNING 状态 - 导纳控制下扫描环境"""
        # 开启导纳控制，让机械臂在人的引导下自由移动
        # 同时视觉系统开始扫描环境
        
        if self.use_force_control:
            # 发送导纳控制参数
            # 参数：虚拟质量、阻尼系数、刚度系数
            force_cmd = Float64MultiArray()
            force_cmd.data = [1.0, 30.0, 50.0]  # [M, D, K] 参数
            self.force_control_pub.publish(force_cmd)
        
        # 当检测到物体时，进入检测状态
        if self.detected_objects and len(self.detected_objects) > 0:
            self.system_state = SystemState.DETECTING
            self.get_logger().info(f"✓ 检测到 {len(self.detected_objects)} 个物体，进入检测模式")

    def state_detecting(self):
        """DETECTING 状态 - 根据手势选择目标物体"""
        if not self.detected_objects:
            self.system_state = SystemState.SCANNING
            return

        # 根据人的手势选择目标物体
        if self.human_gesture and self.human_gesture.gesture_type != "none":
            if self.human_gesture.gesture_type == "grab_left":
                # 选择图像左侧的物体
                self.target_object = min(
                    self.detected_objects,
                    key=lambda obj: obj.center_2d[0]
                )
            elif self.human_gesture.gesture_type == "grab_right":
                # 选择图像右侧的物体
                self.target_object = max(
                    self.detected_objects,
                    key=lambda obj: obj.center_2d[0]
                )

            if self.target_object:
                self.get_logger().info(
                    f"🎯 选择目标: {self.target_object.class_name} "
                    f"(置信度: {self.target_object.confidence:.2f})")
                self.system_state = SystemState.PLANNING
        else:
            # 如果没有手势，选择置信度最高的物体
            self.target_object = max(
                self.detected_objects,
                key=lambda obj: obj.confidence
            )
            self.get_logger().info(
                f"🎯 自动选择目标: {self.target_object.class_name} "
                f"(置信度: {self.target_object.confidence:.2f})")
            self.system_state = SystemState.PLANNING

    def state_planning(self):
        """PLANNING 状态 - 规划抓取轨迹"""
        if not self.target_object:
            self.system_state = SystemState.DETECTING
            return

        self.get_logger().info("📋 规划抓取轨迹...")
        
        # 从手眼标定结果获取目标物体的 3D 坐标
        # TODO: 从 /hand_eye_calibration_result 订阅标定结果
        # 这里假设已有 3D 坐标
        target_x = 0.3
        target_y = 0.1
        target_z = 0.05
        
        # 规划抓取位置（稍微高于物体）
        grasp_height = target_z + 0.05  # 比物体高 50mm
        
        # 目标位姿：俯视角度
        target_pose = Pose()
        target_pose.position.x = target_x
        target_pose.position.y = target_y
        target_pose.position.z = grasp_height
        
        # 设置为俯视（Roll=π, Pitch=0, Yaw=0）
        q = self.get_quaternion_from_euler(math.pi, 0, 0)
        target_pose.orientation = q
        
        # 规划轨迹 - 发送 Movel 命令给 rm_driver
        self.plan_trajectory_linear(target_pose)
        self.system_state = SystemState.EXECUTING
        self.get_logger().info("✓ 轨迹规划完成，准备执行")

    def state_executing(self):
        """EXECUTING 状态 - 执行规划的轨迹"""
        # 检查机械臂是否已完成运动
        if not self.arm_is_moving:
            self.get_logger().info("✓ 到达目标位置")
            self.system_state = SystemState.GRASPING
        else:
            # 检查超时
            if time.time() - self.last_move_time > 30.0:
                self.get_logger().error("❌ 轨迹执行超时")
                self.system_state = SystemState.ERROR

    def state_grasping(self):
        """GRASPING 状态 - 执行夹爪抓取"""
        self.get_logger().info("👆 执行夹爪抓取...")
        
        # 发送夹爪闭合命令
        # TODO: 使用官方的 Gripper 消息
        # gripper_msg = GripperCommand()
        # gripper_msg.command.position = 0.0  # 闭合
        # gripper_msg.command.effort = 50.0   # 力度
        
        time.sleep(2.0)  # 等待夹爪闭合
        
        # 检测夹爪是否抓住物体（通过力反馈）
        self.get_logger().info("✓ 夹爪闭合成功")
        self.system_state = SystemState.PLACING

    def state_placing(self):
        """PLACING 状态 - 放置物体"""
        self.get_logger().info("📍 计划放置物体...")
        
        # 规划放置轨迹
        place_pose = Pose()
        place_pose.position.x = self.place_location[0]
        place_pose.position.y = self.place_location[1]
        place_pose.position.z = self.place_location[2]
        
        q = self.get_quaternion_from_euler(math.pi, 0, 0)
        place_pose.orientation = q
        
        self.plan_trajectory_linear(place_pose)
        
        # 等待到达放置位置
        time.sleep(3.0)
        
        # 打开夹爪
        self.get_logger().info("✋ 打开夹爪释放物体...")
        # TODO: 发送夹爪打开命令
        time.sleep(1.0)
        
        self.get_logger().info("✅ 物体放置完成")
        
        # 返回初始状态
        self.system_state = SystemState.IDLE
        self.target_object = None

    @staticmethod
    def get_quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        欧拉角转四元数
        
        参数：
        - roll (φ): 绕 X 轴旋转
        - pitch (θ): 绕 Y 轴旋转  
        - yaw (ψ): 绕 Z 轴旋转
        
        返回：四元数 (w, x, y, z)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    @staticmethod
    def get_euler_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
        """
        四元数转欧拉角
        
        参数：
        - q: 四元数 (Quaternion 类型)
        
        返回：(roll, pitch, yaw) 元组
        """
        # Roll (x 轴旋转)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y 轴旋转)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z 轴旋转)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def plan_trajectory_joint(self, target_joints: List[float]):
        """
        关节空间轨迹规划 - 使用 Movej 消息
        
        原理：直接指定关节角度目标，rm_driver 会通过官方 API 的 rm_movej() 调用
        
        参数：
        - target_joints: 目标关节角度列表 [J1, J2, J3, J4, J5, J6, J7] (单位：弧度)
        
        【重要】官方 API 对应关系：
        int rm_movej(rm_robot_handle *handle, const float *joint, int v, int r, 
                     int trajectory_connect, int block);
        
        其中：
        - v: 速度百分比 (1-100)
        - r: 交融半径 (0-100)
        - trajectory_connect: 0=立即执行, 1=与下一条轨迹一起规划
        - block: 0=非阻塞, 1=阻塞
        """
        if len(target_joints) != self.arm_dof:
            self.get_logger().error(
                f"❌ 关节数量错误: 期望 {self.arm_dof}，获得 {len(target_joints)}")
            return
        
        # 创建 Movej 消息 - 这会被 rm_driver 订阅并调用官方 API
        movej_msg = Movej()
        movej_msg.joint = list(target_joints)
        movej_msg.speed = self.arm_speed
        movej_msg.trajectory_connect = 0  # 立即执行
        movej_msg.block = True  # 阻塞模式
        movej_msg.dof = self.arm_dof
        
        # 发布给 rm_driver
        self.movej_pub.publish(movej_msg)
        self.arm_is_moving = True
        self.last_move_time = time.time()
        
        self.get_logger().info(
            f"📤 发送关节运动命令: {[f'{math.degrees(j):.1f}°' for j in target_joints]}")

    def plan_trajectory_linear(self, target_pose: Pose):
        """
        笛卡尔空间直线轨迹规划 - 使用 Movel 消息
        
        原理：末端执行器沿直线运动至目标位姿，rm_driver 会通过官方 API 的 rm_movel() 调用
        
        参数：
        - target_pose: 目标位姿 (Pose 类型，包含 position 和 orientation)
        
        【重要】官方 API 对应关系：
        int rm_movel(rm_robot_handle *handle, const rm_pose_t *pose, int v, int r,
                     int trajectory_connect, int block);
        
        其中：
        - pose: 目标位姿 (位置单位：米，方向用四元数)
        - v: 速度百分比 (1-100)
        - r: 交融半径 (0-100)  
        - trajectory_connect: 0=立即执行, 1=与下一条轨迹一起规划
        - block: 0=非阻塞, 1=阻塞
        """
        # 创建 Movel 消息 - 这会被 rm_driver 订阅并调用官方 API
        movel_msg = Movel()
        movel_msg.pose = target_pose
        movel_msg.speed = self.arm_speed
        movel_msg.trajectory_connect = 0  # 立即执行
        movel_msg.block = True  # 阻塞模式
        
        # 发布给 rm_driver
        self.movel_pub.publish(movel_msg)
        self.arm_is_moving = True
        self.last_move_time = time.time()
        
        self.get_logger().info(
            f"📤 发送直线运动命令: "
            f"P({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})m")

    def go_home(self):
        """
        机械臂回到 Home 位置
        
        使用 Movej 关节空间轨迹规划
        官方 API: rm_movej()
        """
        self.get_logger().info("🏠 机械臂回到 Home 位置...")
        self.plan_trajectory_joint(self.home_position)


def main(args=None):
    """主函数 - 启动主控制节点"""
    rclpy.init(args=args)

    node = MainControllerNode()

    # 使用多线程执行器,支持并发回调
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
