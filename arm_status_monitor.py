#!/usr/bin/env python3
"""
机械臂状态监控脚本
实时显示6轴角度、空间位姿和夹爪状态
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import math


class ArmStatusMonitor(Node):
    def __init__(self):
        super().__init__('arm_status_monitor')
        
        # 订阅关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # TF监听器用于获取末端位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 状态变量
        self.current_joint_positions = None
        self.current_joint_velocities = None
        self.current_end_effector_pose = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 定时器用于更新显示
        self.display_timer = self.create_timer(1.0, self.display_status)
        
        self.get_logger().info('🤖 机械臂状态监控器已启动')
        print("\n" + "="*70)
        print("🤖 机械臂状态监控器")
        print("="*70)

    def joint_state_callback(self, msg):
        """接收关节状态消息"""
        if len(msg.position) >= 6:
            self.current_joint_positions = np.array(msg.position[:6])
            if msg.velocity:
                self.current_joint_velocities = np.array(msg.velocity[:6])
            
            # 获取末端执行器位姿
            self.get_end_effector_pose()

    def get_end_effector_pose(self):
        """获取末端执行器相对于基座的位姿"""
        try:
            # 尝试获取从base_link到link6_1_1的变换
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'link6_1_1', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            self.current_end_effector_pose = transform
        except Exception as e:
            # 如果TF变换不可用，不更新位姿
            pass

    def rad_to_deg(self, rad_array):
        """弧度转角度"""
        return rad_array * 180.0 / math.pi

    def display_status(self):
        """显示当前状态"""
        if self.current_joint_positions is None:
            print("⏳ 等待机械臂数据...")
            return
        
        # 清屏并显示状态
        print("\033[2J\033[H")  # 清屏
        print("="*70)
        print("🤖 机械臂实时状态监控")
        print(f"⏰ 更新时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*70)
        
        # 显示关节角度
        print("\n📐 关节角度:")
        print("-" * 50)
        joint_degrees = self.rad_to_deg(self.current_joint_positions)
        
        for i, (name, deg, rad) in enumerate(zip(self.joint_names, joint_degrees, self.current_joint_positions)):
            vel_str = ""
            if self.current_joint_velocities is not None:
                vel_deg_s = self.rad_to_deg(self.current_joint_velocities[i])
                vel_str = f" | 速度: {vel_deg_s:6.2f}°/s"
            
            print(f"  {name}: {deg:8.2f}° ({rad:6.3f} rad){vel_str}")
        
        # 显示末端执行器位姿
        print("\n🎯 末端执行器位姿 (相对于base_link):")
        print("-" * 50)
        
        if self.current_end_effector_pose:
            pos = self.current_end_effector_pose.transform.translation
            rot = self.current_end_effector_pose.transform.rotation
            
            # 位置
            print(f"  位置 (m):")
            print(f"    X: {pos.x:8.4f}")
            print(f"    Y: {pos.y:8.4f}")
            print(f"    Z: {pos.z:8.4f}")
            
            # 四元数姿态
            print(f"  姿态 (四元数):")
            print(f"    X: {rot.x:8.4f}")
            print(f"    Y: {rot.y:8.4f}")
            print(f"    Z: {rot.z:8.4f}")
            print(f"    W: {rot.w:8.4f}")
            
            # 转换为欧拉角 (RPY)
            rpy = self.quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)
            print(f"  姿态 (欧拉角):")
            print(f"    Roll:  {math.degrees(rpy[0]):8.2f}°")
            print(f"    Pitch: {math.degrees(rpy[1]):8.2f}°")
            print(f"    Yaw:   {math.degrees(rpy[2]):8.2f}°")
        else:
            print("  ⚠️  末端位姿信息不可用 (TF变换未找到)")
        
        # 显示工作空间信息
        print("\n📊 工作空间信息:")
        print("-" * 50)
        if self.current_end_effector_pose:
            pos = self.current_end_effector_pose.transform.translation
            distance_from_base = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
            print(f"  距离基座: {distance_from_base:6.3f} m")
            print(f"  工作半径: {math.sqrt(pos.x**2 + pos.y**2):6.3f} m")
            print(f"  工作高度: {pos.z:6.3f} m")
        
        print("\n💡 按 Ctrl+C 退出监控")
        print("="*70)

    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角 (RPY)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = ArmStatusMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n🛑 监控器已停止")
    finally:
        try:
            monitor.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 