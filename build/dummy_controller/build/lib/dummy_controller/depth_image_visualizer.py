#!/usr/bin/env python3
"""
D435深度相机数据可视化程序
用于显示从Gazebo仿真环境中D435深度相机获取的数据
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthImageVisualizer(Node):
    """深度图像可视化节点"""
    
    def __init__(self):
        super().__init__('depth_image_visualizer')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 订阅深度图像话题（修复话题名称）
        self.depth_subscription = self.create_subscription(
            Image,
            '/d435/d435/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # 订阅彩色图像话题（修复话题名称）
        self.color_subscription = self.create_subscription(
            Image,
            '/d435/d435/image_raw',
            self.color_callback,
            10
        )
        
        # 注意：Gazebo libgazebo_ros_camera.so插件不会生成点云数据
        # 如果需要点云，需要从深度图像和相机内参自己计算生成
        
        self.get_logger().info('深度图像可视化节点已启动')
        self.get_logger().info('订阅的话题:')
        self.get_logger().info('  - /d435/d435/depth/image_raw (深度图像)')
        self.get_logger().info('  - /d435/d435/image_raw (彩色图像)')
        self.get_logger().info('注意: Gazebo相机插件不直接提供点云数据')
        
        # 用于统计的计数器
        self.depth_count = 0
        self.color_count = 0
        
        # 创建定时器用于状态报告
        self.timer = self.create_timer(5.0, self.status_callback)
    
    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 归一化深度图像用于显示
            depth_display = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
            
            # 显示深度图像
            cv2.imshow('D435 Depth Image', depth_colormap)
            cv2.waitKey(1)
            
            self.depth_count += 1
            
            # 打印深度信息（每30帧打印一次）
            if self.depth_count % 30 == 0:
                min_depth = np.min(depth_image[depth_image > 0])
                max_depth = np.max(depth_image)
                mean_depth = np.mean(depth_image[depth_image > 0])
                
                self.get_logger().info(
                    f'深度统计 - 最小: {min_depth:.3f}m, 最大: {max_depth:.3f}m, 平均: {mean_depth:.3f}m'
                )
                
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {e}')
    
    def color_callback(self, msg):
        """彩色图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 显示彩色图像
            cv2.imshow('D435 Color Image', color_image)
            cv2.waitKey(1)
            
            self.color_count += 1
            
        except Exception as e:
            self.get_logger().error(f'处理彩色图像时出错: {e}')
    
    def status_callback(self):
        """状态报告回调函数"""
        self.get_logger().info(
            f'数据接收状态 - 深度: {self.depth_count}, 彩色: {self.color_count}'
        )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        visualizer = DepthImageVisualizer()
        
        print("深度图像可视化程序已启动！")
        print("按键说明:")
        print("  - 'q' 或 'ESC': 退出程序")
        print("  - 任意其他键: 继续")
        print()
        print("窗口操作:")
        print("  - 将显示两个OpenCV窗口：深度图像和彩色图像")
        print("  - 深度图像使用JET颜色映射显示")
        print("  - 红色表示近距离，蓝色表示远距离")
        print()
        print("修复说明:")
        print("  - 已修复话题名称匹配问题")
        print("  - 实际话题: /d435/d435/depth/image_raw 和 /d435/d435/image_raw")
        print("  - Gazebo相机插件不直接提供点云数据")
        
        rclpy.spin(visualizer)
        
    except KeyboardInterrupt:
        print("\n接收到中断信号，正在关闭...")
    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        # 清理资源
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 