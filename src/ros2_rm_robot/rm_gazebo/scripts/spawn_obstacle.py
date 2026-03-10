#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
import time

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')

        # 1. 发布到 /planning_scene 话题
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # 2. 等待 move_group 节点出现
        self.get_logger().info('Waiting for move_group node...')
        while 'move_group' not in self.get_node_names():
            time.sleep(1.0)

        # 3. 构造 CollisionObject（盒子）
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'   # 与 SRDF 里虚拟关节 parent 一致
        obj.id = 'box_obstacle'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1, 0.2, 0.1]  # x y z

        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.3
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        # 4. 构造 PlanningScene 消息
        scene = PlanningScene()
        scene.world.collision_objects.append(obj)
        scene.is_diff = True

        # 5. 发布并等待生效
        self.get_logger().info('Publishing obstacle to /planning_scene ...')
        self.scene_pub.publish(scene)
        time.sleep(2.0)   # 给 RViz 一点接收时间
        self.get_logger().info('Obstacle added.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    rclpy.spin_once(node, timeout_sec=0.1)  # 只 spin 一次即可
    rclpy.shutdown()

if __name__ == '__main__':
    main()