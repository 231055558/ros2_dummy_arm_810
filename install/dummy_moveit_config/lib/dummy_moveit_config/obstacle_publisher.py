#!/usr/bin/env python3
"""
障碍物发布节点
向MoveIt planning scene发布静态障碍物，用于运动规划避障
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
import time


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        
        # 创建planning scene发布器
        self.scene_publisher = self.create_publisher(
            PlanningScene, 
            'planning_scene', 
            10
        )
        
        # 等待发布器建立连接
        time.sleep(2.0)
        
        # 发布初始障碍物
        self.publish_obstacles()
        
        # 定时发布障碍物（确保持续存在）
        self.timer = self.create_timer(5.0, self.publish_obstacles)
        
        self.get_logger().info('障碍物发布节点已启动')

    def create_box_obstacle(self, name, x, y, z, size_x, size_y, size_z, frame_id="world"):
        """
        创建一个盒子形状的障碍物
        
        参数:
        - name: 障碍物名称
        - x, y, z: 障碍物位置
        - size_x, size_y, size_z: 障碍物尺寸
        - frame_id: 坐标系
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = name

        # 创建盒子形状
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size_x, size_y, size_z]

        # 设置障碍物位置
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        # 添加到collision object
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        return collision_object

    def create_cylinder_obstacle(self, name, x, y, z, radius, height, frame_id="world"):
        """
        创建一个圆柱形障碍物
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = name

        # 创建圆柱形状
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]  # [height, radius]

        # 设置障碍物位置
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        # 添加到collision object
        collision_object.primitives.append(cylinder)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        return collision_object

    def publish_obstacles(self):
        """发布障碍物到planning scene"""
        
        # 创建planning scene消息
        planning_scene = PlanningScene()
        planning_scene.is_diff = True  # 这是一个差分更新

        # 示例障碍物1：桌子（盒子形状）
        table = self.create_box_obstacle(
            name="table",
            x=0.5,   # 机械臂前方50cm
            y=0.0,   # 正前方
            z=-0.1,  # 基座下方10cm（桌面高度）
            size_x=1.0,  # 长度1米
            size_y=0.8,  # 宽度80cm
            size_z=0.02  # 厚度2cm
        )

        # 示例障碍物2：立柱（圆柱形状）
        pillar = self.create_cylinder_obstacle(
            name="pillar",
            x=0.3,     # 机械臂前方30cm
            y=0.4,     # 右侧40cm
            z=0.4,     # 高度40cm
            radius=0.05,  # 半径5cm
            height=0.8    # 高度80cm
        )

        # 示例障碍物3：防护墙（盒子形状）
        wall = self.create_box_obstacle(
            name="safety_wall",
            x=0.0,     # 机械臂侧面
            y=0.6,     # 右侧60cm
            z=0.25,    # 高度25cm
            size_x=0.02,  # 厚度2cm
            size_y=0.1,   # 宽度10cm
            size_z=0.5    # 高度50cm
        )

        # 将障碍物添加到planning scene
        planning_scene.world.collision_objects = [table, pillar, wall]

        # 发布planning scene
        self.scene_publisher.publish(planning_scene)
        
        self.get_logger().info(f'已发布 {len(planning_scene.world.collision_objects)} 个障碍物')


def main(args=None):
    rclpy.init(args=args)
    
    obstacle_publisher = ObstaclePublisher()
    
    try:
        rclpy.spin(obstacle_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 