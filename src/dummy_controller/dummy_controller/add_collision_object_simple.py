#!/usr/bin/env python3
"""
简单的碰撞对象添加程序
不依赖moveit_commander，使用基础ROS 2接口
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import time


class SimpleCollisionObjectAdder(Node):
    def __init__(self):
        super().__init__('simple_collision_object_adder')
        
        # 创建碰撞对象发布器
        self.collision_object_pub = self.create_publisher(
            CollisionObject, 
            '/collision_object', 
            10
        )
        
        # 创建规划场景发布器
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        self.get_logger().info("简单碰撞对象添加器已启动")
        
        # 等待发布器就绪
        time.sleep(1.0)
        
        # 添加不同类型的碰撞对象
        self.add_table()
        self.add_wall()
        self.add_sphere()
        
    def add_table(self):
        """添加一个桌子碰撞对象"""
        collision_object = CollisionObject()
        
        # 设置header
        collision_object.header = Header()
        collision_object.header.frame_id = "base_link"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        
        # 设置对象ID和操作
        collision_object.id = "table"
        collision_object.operation = CollisionObject.ADD
        
        # 创建盒子形状
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.8, 1.2, 0.05]  # 长、宽、高
        
        # 设置位置
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = -0.1  # 桌面在机械臂下方
        pose.orientation.w = 1.0
        
        # 添加到碰撞对象
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        
        # 发布
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info("已添加桌子碰撞对象")
        
    def add_wall(self):
        """添加一个墙壁碰撞对象"""
        collision_object = CollisionObject()
        
        collision_object.header = Header()
        collision_object.header.frame_id = "base_link"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        
        collision_object.id = "wall"
        collision_object.operation = CollisionObject.ADD
        
        # 创建墙壁盒子
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 2.0, 1.5]  # 薄墙
        
        pose = Pose()
        pose.position.x = 0.6  # 在机械臂前方
        pose.position.y = 0.0
        pose.position.z = 0.75
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info("已添加墙壁碰撞对象")
        
    def add_sphere(self):
        """添加一个球形碰撞对象"""
        collision_object = CollisionObject()
        
        collision_object.header = Header()
        collision_object.header.frame_id = "base_link"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        
        collision_object.id = "sphere"
        collision_object.operation = CollisionObject.ADD
        
        # 创建球体
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.1]  # 半径
        
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.3
        pose.position.z = 0.3
        pose.orientation.w = 1.0
        
        collision_object.primitives.append(sphere)
        collision_object.primitive_poses.append(pose)
        
        self.collision_object_pub.publish(collision_object)
        self.get_logger().info("已添加球形碰撞对象")
        
    def remove_all_objects(self):
        """移除所有碰撞对象"""
        objects_to_remove = ["table", "wall", "sphere"]
        
        for obj_id in objects_to_remove:
            collision_object = CollisionObject()
            collision_object.header = Header()
            collision_object.header.frame_id = "base_link"
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = obj_id
            collision_object.operation = CollisionObject.REMOVE
            
            self.collision_object_pub.publish(collision_object)
            
        self.get_logger().info("已移除所有碰撞对象")


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleCollisionObjectAdder()
    
    try:
        # 让节点运行一段时间以确保消息被发送
        rate = node.create_rate(1)
        for i in range(5):
            rclpy.spin_once(node, timeout_sec=1.0)
            rate.sleep()
            
        node.get_logger().info("碰撞对象添加完成")
        
    except KeyboardInterrupt:
        node.get_logger().info("程序被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 