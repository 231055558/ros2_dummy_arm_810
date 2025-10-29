# ---------------------------------------------------------------------------
# 0. 导入所有必要的库
# ---------------------------------------------------------------------------
import os
import json
import math
import sys
from time import sleep
from pathlib import Path
from typing import List, Optional, Dict
from pathlib import Path 

from zhipuai import ZhipuAiClient
from dotenv import load_dotenv
from llm import plan_robot_tasks
from vlm import VLMDetector
from stt import SpeechToText 

import rclpy
from simple_moveit_controller import SimpleMoveItController # 直接导入你的控制器类

import cv2
import pyrealsense2 as rs
import numpy as np

load_dotenv()

# ---------------------------------------------------------------------------
# 1. 【校准与机器人配置区】
# ---------------------------------------------------------------------------
CALIBRATION_CONFIG = {
    "image_width": 1920,
    "image_height": 1080,
    "point1_px": [30, 957],
    "point1_world": [-0.12583986403766642, -0.35024648245505974],
    "point2_px": [1469, 120],
    "point2_world": [0.1830301444979457, -0.19055072363908145]
}

# 抓取/放置平面的Z轴高度（米）
TARGET_Z_HEIGHT = 0.12

# 视觉定位目标的默认姿态 RPY (Roll, Pitch, Yaw) in radians
# [pi/2, 0, 0] 意味着末端工具大致朝下
TARGET_ORIENTATION_RPY = [math.pi / 2, 0, 0]

# 抓取时额外下降的深度（米）
PICK_DEPTH = 0.03

# 预定义的机器人姿态（关节角度，单位：度）
PREDEFINED_POSES = {
    "observation_pose": [0, 0, 0, 0, -90, 0],
    "home_pose": [0, 0, 0, 0, 0, 0]
}


# ---------------------------------------------------------------------------
# 2. 辅助类与函数
# ---------------------------------------------------------------------------
class CameraCalibrator:
    # ... (这部分代码与之前完全相同，无需修改) ...
    def __init__(self, **config):
        self.img_w = config["image_width"]
        self.img_h = config["image_height"]
        px1, py1 = config["point1_px"]; wx1, wy1 = config["point1_world"]
        px2, py2 = config["point2_px"]; wx2, wy2 = config["point2_world"]
        if (px2 - px1) == 0 or (py2 - py1) == 0: raise ValueError("校准点像素坐标不能在同一直线！")
        self.scale_x = (wx2 - wx1) / (px2 - px1); self.offset_x = wx1 - self.scale_x * px1
        self.scale_y = (wy2 - wy1) / (py2 - py1); self.offset_y = wy1 - self.scale_y * py1
        print("[Calibrator] 相机校准完成。")

    def convert(self, px: float, py: float) -> List[float]:
        return [self.scale_x * px + self.offset_x, self.scale_y * py + self.offset_y]

# ---------------------------------------------------------------------------
# 3. 【已整合】真实的机器人控制器适配器
# ---------------------------------------------------------------------------
class RobotController:
    """
    高级机器人控制器 (适配器)。
    将Agent的简单指令翻译成SimpleMoveItController的具体ROS2调用。
    """
    def __init__(self, ros_controller_node: SimpleMoveItController):
        # 包含一个真实的ROS2节点控制器实例
        self.controller = ros_controller_node
        print("[Robot] RobotController 已链接到 ROS2 节点。")

    def move_to_location(self, location_name: str, memorized_locations: Dict[str, List[float]]) -> bool:
        print(f"[Robot] 收到移动指令，目标: '{location_name}'")
        
        # Case 1: 目标是预定义的关节姿态 (points)
        if location_name in PREDEFINED_POSES:
            joint_angles_deg = PREDEFINED_POSES[location_name]
            print(f"[Robot] 目标为预设关节姿态。正在移动到: {joint_angles_deg} deg...")
            return self.controller.move_to_joint_positions(joint_angles_deg, unit='deg')
        
        # Case 2: 目标是记忆中的笛卡尔位姿 (pose)
        elif location_name in memorized_locations:
            xyz = memorized_locations[location_name]
            x, y, z = xyz
            rx, ry, rz = TARGET_ORIENTATION_RPY
            print(f"[Robot] 目标为记忆位姿。正在移动到 XYZ: [{x:.3f}, {y:.3f}, {z:.3f}], RPY: [{rx:.3f}, {ry:.3f}, {rz:.3f}]...")
            return self.controller.move_to_cartesian_pose(x, y, z, rx, ry, rz)
            
        else:
            print(f"!!! [Robot] 错误: 未知的目标位置 '{location_name}'")
            return False

    def capture_and_save_image(self) -> str:
        print("[Robot] 正在通过 RealSense 拍照...")
        image_path = "latest_capture.jpg"
        pipeline = None
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, CALIBRATION_CONFIG['image_width'], CALIBRATION_CONFIG['image_height'], rs.format.bgr8, 30)
            pipeline.start(config)
            
            # 等待自动曝光/白平衡稳定
            for _ in range(10):
                pipeline.wait_for_frames()
            
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                raise RuntimeError("无法从 RealSense 获取彩色图像帧。")
            
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imwrite(image_path, color_image)
            print(f"[Robot] 图片已成功保存至 '{image_path}'。")
            return image_path
        
        except Exception as e:
            print(f"!!! [Robot] 拍照失败: {e}")
            return ""
        finally:
            if pipeline:
                pipeline.stop()

    def perform_pick(self):
        print("[Robot] 正在执行抓取序列...")
        current_pose = self.controller.get_current_ee_pose()
        if not current_pose:
            print("!!! [Robot] 无法获取当前位姿，抓取中止。")
            return False
        
        x, y, z, rx, ry, rz = current_pose
        
        # 1. 向下移动一小段距离
        print(f"[Robot] 从 Z={z:.3f} 向下移动 {PICK_DEPTH} 米...")
        success = self.controller.move_to_cartesian_pose(x, y, z - PICK_DEPTH, rx, ry, rz)
        if not success:
            print("!!! [Robot] 向下移动失败，抓取中止。")
            return False

        # 2. 闭合夹爪
        print("[Robot] 闭合夹爪...")
        success = self.controller.control_gripper(close_gripper=True)
        if not success:
             print("!!! [Robot] 闭合夹爪失败。")
             return False
        
        sleep(1.0) # 等待夹爪稳定
        print("[Robot] 抓取完成。")
        return True

    def perform_drop(self):
        print("[Robot] 正在执行放置动作...")
        success = self.controller.control_gripper(close_gripper=False)
        sleep(1.0) # 等待夹爪张开
        return success
        
    def shutdown(self):
        print("[Robot] 正在安全关闭机器人...")
        self.controller.enable_robot(False)
        self.controller.destroy_node()


# ---------------------------------------------------------------------------
# 4. 【核心】任务编排Agent
# ---------------------------------------------------------------------------
class TaskOrchestrator:
    def __init__(self, ros_controller_node: SimpleMoveItController):
        self.robot = RobotController(ros_controller_node)
        self.vlm = VLMDetector()
        self.calibrator = CameraCalibrator(**CALIBRATION_CONFIG)
        self.memory = {}
        self.last_image_path = None
        print("[Orchestrator] Agent 已初始化，所有模块已加载。")

    def convert_bbox_to_world_coords(self, bbox: list) -> list:
        xmin, ymin, xmax, ymax = bbox
        norm_cx = (xmin + xmax) / 2.0; norm_cy = (ymin + ymax) / 2.0
        pixel_cx = norm_cx * self.calibrator.img_w; pixel_cy = norm_cy * self.calibrator.img_h
        print(f"[Orchestrator] Bbox中心点: 像素坐标 ({pixel_cx:.2f}, {pixel_cy:.2f})")
        world_x, world_y = self.calibrator.convert(pixel_cx, pixel_cy)
        world_coords = [world_x, world_y, TARGET_Z_HEIGHT]
        print(f"[Orchestrator] 转换完成: 世界坐标 ({world_x:.4f}, {world_y:.4f}, {TARGET_Z_HEIGHT})")
        return world_coords

    def execute_command(self, user_command: str):
        print("="*40); print(f"执行新指令: '{user_command}'"); print("="*40)

        plan = plan_robot_tasks(user_command)
        if not plan:
            print("[Orchestrator] 无法生成任务计划，任务中止。")
            return

        for i, task in enumerate(plan):
            task_type, params = task["task_type"], task["parameters"]
            print(f"\n--- 执行第 {i+1} 步: {task_type}, 参数: {params} ---")
            
            try:
                if task_type == "locate_and_memorize":
                    if not self.last_image_path: 
                        raise Exception("没有可供分析的图片。")
                    
                    description, key = params['target_description'], params['memory_key']

                    image_file_path = Path(self.last_image_path)
                    bbox = self.vlm.get_object_bbox(description, image_file_path)
                    
                    if bbox:
                        coords = self.convert_bbox_to_world_coords(bbox)
                        self.memory[key] = coords
                        print(f"[Orchestrator] 成功！已将 '{description}' 的位置 {coords} 存入记忆: '{key}'。")
                    else: 
                        raise Exception(f"视觉模型未能找到 '{description}'")

                elif task_type == "move_to_location":
                    name = params['location_name']
                    if not self.robot.move_to_location(name, self.memory):
                        raise Exception(f"机器人移动到 '{name}' 失败。")
                    if name == "observation_pose":
                        self.last_image_path = self.robot.capture_and_save_image()
                        if not self.last_image_path: raise Exception("拍照失败。")
                        
                elif task_type == "pick":
                    if not self.robot.perform_pick(): raise Exception("抓取动作失败。")
                    
                elif task_type == "drop":
                    if not self.robot.perform_drop(): raise Exception("放置动作失败。")

                else: print(f"警告：未知的任务类型 '{task_type}'，已跳过。")

            except Exception as e:
                print(f"!!! 错误：在执行第 {i+1} 步 ({task_type}) 时发生: {e}")
                print("!!! 任务计划中止。")
                return # 在 main 的 finally 中关闭
        
        print("\n[Orchestrator] 任务计划已成功完成！")

# ---------------------------------------------------------------------------
# 5. 【已整合语音】主程序入口
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    ros_controller = SimpleMoveItController()
    
    agent = None
    stt_module = None
    try:
        # --- 模块初始化 ---
        stt_module = SpeechToText() # 初始化语音识别模块
        agent = TaskOrchestrator(ros_controller_node=ros_controller)
        
        # --- 机器人和服务准备 ---
        if not ros_controller.wait_ready():
            raise RuntimeError("ROS2/MoveIt 服务初始化失败。")
        if not ros_controller.enable_robot(True):
            raise RuntimeError("使能机械臂失败。")
        
        # --- 主循环，可以持续接收语音指令 ---
        while rclpy.ok():
            # 1. 使用语音识别模块获取指令
            command = stt_module.listen(duration=7) # 增加录音时长到7秒，给用户更多思考时间

            if command:
                print("\n" + "#"*40)
                print(f"##  最终识别到的指令: '{command}'  ##")
                print("#"*40)
                
                # 2. 执行指令
                agent.execute_command(command)
                print("\n任务流程结束。准备接收下一条指令...")
            else:
                print("\n未能识别到有效指令，请重试。")

            # 询问是否继续
            continue_prompt = input("按 Enter 键继续下达指令，或输入 'q' 退出: ")
            if continue_prompt.lower() == 'q':
                break
    
    except KeyboardInterrupt:
        print("\n程序被用户中断。")
    except Exception as e:
        error_logger = agent.robot.controller.get_logger() if agent else print
        error_logger(f"Agent执行中发生严重错误: {e}")
    finally:
        # --- 安全关闭 ---
        if agent:
            agent.robot.shutdown()
        elif 'ros_controller' in locals() and ros_controller:
             ros_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("程序已安全退出。")

if __name__ == "__main__":
    main()