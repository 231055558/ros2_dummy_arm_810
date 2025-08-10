#!/usr/bin/env python3
"""
直接运行的障碍物添加脚本
添加三个长方体障碍物到MoveIt planning scene
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
import time
import sys


def create_box_obstacle(name, x, y, z, size_x, size_y, size_z, frame_id="world"):
    """创建长方体障碍物"""
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = name

    # 创建长方体
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [size_x, size_y, size_z]  # 长方体，三个维度可以不同

    # 设置位置
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    return collision_object


def create_right_trapezoid_prism(name, center_x, center_y, center_z,
                                  bottom_width, top_width, height, thickness,
                                  frame_id="world"):
    """创建一个立着的直角梯形柱体（沿 x 方向挤出 thickness），侧视图在 y-z 平面为直角梯形，直角边竖直对地面。

    定义：
    - z 轴为竖直方向（高度）
    - y 轴为前后方向（梯形的平行边方向）
    - x 轴为厚度挤出方向
    - 直角边在 y = -max(bottom_width, top_width)/2 处竖直（与地面垂直）
    """
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = name

    h = float(height)
    t = float(thickness)
    bw = float(bottom_width)
    tw = float(top_width)

    maxw = max(bw, tw)
    y_left = -maxw / 2.0

    # 顶/底边中心对齐到 y 的包围盒中心，左侧为直角竖边
    # 顶点顺序（局部网格坐标）：先 x = -t/2 面，再 x = +t/2 面
    # 后面（x = -t/2）四点：
    v0 = Point(x=-t/2.0, y=y_left,              z=-h/2.0)            # 底-左（直角）
    v1 = Point(x=-t/2.0, y=y_left + bw,         z=-h/2.0)            # 底-右
    v2 = Point(x=-t/2.0, y=y_left + tw,         z= h/2.0)            # 顶-右
    v3 = Point(x=-t/2.0, y=y_left,              z= h/2.0)            # 顶-左（直角）

    # 前面（x = +t/2）四点：
    v4 = Point(x= t/2.0, y=y_left,              z=-h/2.0)
    v5 = Point(x= t/2.0, y=y_left + bw,         z=-h/2.0)
    v6 = Point(x= t/2.0, y=y_left + tw,         z= h/2.0)
    v7 = Point(x= t/2.0, y=y_left,              z= h/2.0)

    mesh = Mesh()
    mesh.vertices = [v0, v1, v2, v3, v4, v5, v6, v7]

    def tri(a, b, c):
        mt = MeshTriangle()
        mt.vertex_indices[0] = a
        mt.vertex_indices[1] = b
        mt.vertex_indices[2] = c
        return mt

    triangles = []
    # 后侧梯形面（x = -t/2）：v0-v1-v2-v3
    triangles.append(tri(0, 1, 2))
    triangles.append(tri(0, 2, 3))
    # 前侧梯形面（x = +t/2）：v4-v5-v6-v7
    triangles.append(tri(4, 5, 6))
    triangles.append(tri(4, 6, 7))
    # 底面（z = -h/2）：v0-v1-v5-v4
    triangles.append(tri(0, 1, 5))
    triangles.append(tri(0, 5, 4))
    # 顶面（z = +h/2）：v3-v2-v6-v7
    triangles.append(tri(3, 2, 6))
    triangles.append(tri(3, 6, 7))
    # 左侧直角竖边面：v0-v3-v7-v4
    triangles.append(tri(0, 3, 7))
    triangles.append(tri(0, 7, 4))
    # 右侧斜边面：v1-v2-v6-v5
    triangles.append(tri(1, 2, 6))
    triangles.append(tri(1, 6, 5))

    mesh.triangles = triangles

    pose = Pose()
    pose.position.x = float(center_x)
    pose.position.y = float(center_y)
    pose.position.z = float(center_z)
    pose.orientation.w = 1.0

    collision_object.meshes.append(mesh)
    collision_object.mesh_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    return collision_object


def main():
    rclpy.init()
    
    # 创建临时节点
    node = Node('obstacle_adder')
    
    # 创建发布器
    scene_publisher = node.create_publisher(PlanningScene, 'planning_scene', 10)
    
    # 等待连接建立
    print("等待连接建立...")
    time.sleep(2.0)
    
    # 创建planning scene消息
    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    
    obstacles = []

    # 货柜限制
    # 左侧限制
    shelf_left = create_box_obstacle(
        name="box1",
        x=0.27,     # 机械臂右侧20cm
        y=0.425,     # 前方20cm
        z=0.23,    # 高度5cm
        size_x=0.01,  # 长度10cm
        size_y=0.345, # 宽度34.5cm
        size_z=0.525   # 高度10cm
    )
    obstacles.append(shelf_left)
    
    # 右侧限制
    shelf_right = create_box_obstacle(
        name="box2",
        x=-0.27,     # 机械臂左侧20cm
        y=0.425,     # 前方20cm
        z=0.23,    # 高度5cm
        size_x=0.01,  # 长度10cm
        size_y=0.345, # 宽度34.5cm
        size_z=0.525   # 高度10cm
    )
    obstacles.append(shelf_right)
    
    # 上方限制
    shelf_up = create_box_obstacle(
        name="box3",
        x=0.0,     # 机械臂左侧20cm
        y=0.425,     # 前方20cm
        z=0.47,    # 高度5cm
        size_x=0.54,  # 长度10cm
        size_y=0.345, # 宽度15cm
        size_z=0.01   # 高度10cm
    )
    obstacles.append(shelf_up)

    # 下方限制
    shelf_down = create_box_obstacle(
        name="box4",
        x=0.0,     # 机械臂左侧20cm
        y=0.405,     # 前方20cm
        z=0.20,    # 高度5cm
        size_x=0.54,  # 长度10cm
        size_y=0.3, # 宽度15cm
        size_z=0.01   # 高度10cm
    )
    obstacles.append(shelf_down)

    #杯子
    cup = create_box_obstacle(
        name="cup",
        x=-0.215,
        y=0.36,
        z=0.1525,
        size_x=0.05,
        size_y=0.01,
        size_z=0.05
    )
    obstacles.append(cup)

    #车体限制
    car_body = create_box_obstacle(
        name="car_body",
        x=0.015,
        y=0.04,
        z=-0.0175,
        size_x=0.255,
        size_y=0.34,
        size_z=0.1
    )
    obstacles.append(car_body)
    
    #驱动板
    driver_board = create_box_obstacle(
        name="driver_board",
        x=0.015,
        y=-0.045,
        z=0.0815,
        size_x=0.15,
        size_y=0.09,
        size_z=0.09
    )
    obstacles.append(driver_board)

    # 左侧限制
    cd_left = create_box_obstacle(
        name="cd_left",
        x=0.09,     # 机械臂右侧20cm
        y=-0.07,     # 前方20cm
        z=0.0815,    # 高度5cm
        size_x=0.01,  # 长度10cm
        size_y=0.181, # 宽度34.5cm
        size_z=0.09   # 高度10cm
    )
    obstacles.append(cd_left)

    # 右侧限制
    cd_right = create_box_obstacle(
        name="cd_right",
        x=-0.06,     # 机械臂右侧20cm
        y=-0.07,     # 前方20cm
        z=0.0815,    # 高度5cm
        size_x=0.01,  # 长度10cm
        size_y=0.181, # 宽度34.5cm
        size_z=0.09   # 高度10cm
    )
    obstacles.append(cd_right)

    # 后侧限制
    cd_back = create_box_obstacle(
        name="cd_back",
        x=0.015,     # 机械臂右侧20cm
        y=-0.16,     # 前方20cm
        z=0.0915,    # 高度5cm
        size_x=0.15,  # 长度10cm
        size_y=0.01, # 宽度34.5cm
        size_z=0.13   # 高度10cm
    )
    obstacles.append(cd_back)  


    # 直角梯形柱体（示例参数，可按需调整）
    trapezoid1 = create_right_trapezoid_prism(
        name="trapezoid1",
        center_x=0.09,
        center_y=-0.06,
        center_z=0.134,
        bottom_width=0.181,
        top_width=0.10,
        height=0.024,
        thickness=0.01,
        frame_id="world",
    )
    obstacles.append(trapezoid1)

    # 直角梯形柱体（示例参数，可按需调整）
    trapezoid2 = create_right_trapezoid_prism(
        name="trapezoid2",
        center_x=-0.06,
        center_y=-0.06,
        center_z=0.134,
        bottom_width=0.181,
        top_width=0.10,
        height=0.024,
        thickness=0.01,
        frame_id="world",
    )
    obstacles.append(trapezoid2)
    
    # 添加到planning scene
    planning_scene.world.collision_objects = obstacles
    
    # 发布障碍物
    scene_publisher.publish(planning_scene)
    
    print(f"✅ 已添加 {len(obstacles)} 个障碍物:")
    for i, obs in enumerate(obstacles, 1):
        if len(obs.primitives) > 0 and len(obs.primitive_poses) > 0:
            box = obs.primitives[0]
            pos = obs.primitive_poses[0].position
            print(f"   长方体{i} ({obs.id}): 位置({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), 尺寸({box.dimensions[0]:.2f}×{box.dimensions[1]:.2f}×{box.dimensions[2]:.2f})m")
        elif len(obs.meshes) > 0 and len(obs.mesh_poses) > 0:
            pos = obs.mesh_poses[0].position
            v_count = len(obs.meshes[0].vertices)
            tri_count = len(obs.meshes[0].triangles)
            print(f"   网格{i} ({obs.id}): 位置({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), 顶点{v_count}个, 三角形{tri_count}个")
        else:
            print(f"   障碍物{i} ({obs.id}): 类型未知或数据不完整")
    
    print("\n💡 障碍物已添加到planning scene，在RViz中应该可以看到")
    print("💡 脚本将保持运行5秒以确保消息发送成功...")
    
    # 保持运行一段时间确保消息发送
    time.sleep(5.0)
    
    # 清理
    node.destroy_node()
    rclpy.shutdown()
    
    print("🎯 完成!")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
        sys.exit(1) 