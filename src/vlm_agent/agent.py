# agent/agent.py

import json
from time import sleep
from typing import List, Optional
from pathlib import Path 

# 1. 从你的 llm.py 和 vlm.py 文件中导入函数
# 假设 llm.py 和 vlm.py 在同一个目录下
from llm import plan_robot_tasks
# vlm.py 应该包含一个 VLMDetector 类
from vlm import VLMDetector


# ---------------------------------------------------------------------------
# 【校准配置区】你只需要在这里修改参数
# ---------------------------------------------------------------------------
CALIBRATION_CONFIG = {
    "image_width": 1920,
    "image_height": 1080,
    
    # 校准点 1
    "point1_px": [30, 957],
    "point1_world": [-0.12583986403766642, -0.35024648245505974],
    
    # 校准点 2
    "point2_px": [1469, 120],
    "point2_world": [0.1830301444979457, -0.19055072363908145]
}

# 【重要】定义目标抓取/放置平面的Z轴高度（单位：米）
TARGET_Z_HEIGHT = 0.12

# ---------------------------------------------------------------------------
# 封装校准逻辑的辅助类 (这部分代码你不需要修改)
# ---------------------------------------------------------------------------
class CameraCalibrator:
    """一个根据两点校准数据计算坐标变换的类。"""
    def __init__(self, **config):
        self.img_w = config["image_width"]
        self.img_h = config["image_height"]
        
        px1, py1 = config["point1_px"]
        wx1, wy1 = config["point1_world"]
        px2, py2 = config["point2_px"]
        wx2, wy2 = config["point2_world"]

        if (px2 - px1) == 0 or (py2 - py1) == 0:
            raise ValueError("校准点的像素坐标不能在同一垂直或水平线上！")
            
        self.scale_x = (wx2 - wx1) / (px2 - px1)
        self.offset_x = wx1 - self.scale_x * px1
        self.scale_y = (wy2 - wy1) / (py2 - py1)
        self.offset_y = wy1 - self.scale_y * py1

        print("[Calibrator] 相机校准完成。")
        print(f"[Calibrator]   - Scale (X, Y): ({self.scale_x:.6f}, {self.scale_y:.6f})")
        print(f"[Calibrator]   - Offset (X, Y): ({self.offset_x:.6f}, {self.offset_y:.6f})")

    def convert(self, px: float, py: float) -> List[float]:
        world_x = self.scale_x * px + self.offset_x
        world_y = self.scale_y * py + self.offset_y
        return [world_x, world_y]

# ---------------------------------------------------------------------------
# 【需要你填充】重构后的机器人控制器
# ---------------------------------------------------------------------------
class RobotController:
    """封装所有与ROS2和MoveIt交互的机器人物理动作"""
    def __init__(self):
        print("[Robot] RobotController 初始化完成 (ROS2/MoveIt已连接)。")

    def move_to_location(self, location_name: str, memorized_locations: dict):
        print(f"[Robot] 收到移动指令，目标位置名: '{location_name}'")
        if location_name in ["observation_pose", "home_pose"]:
            print(f"[Robot] 目标为预设姿态。正在移动到 '{location_name}'...")
            # TODO: 实现移动到预设姿态的代码
            sleep(1.5)
            print(f"[Robot] 已到达 '{location_name}'。")
            return True
        elif location_name in memorized_locations:
            world_coords = memorized_locations[location_name]
            print(f"[Robot] 目标为记忆位置。正在移动到坐标: {world_coords}...")
            # TODO: 实现移动到指定世界坐标的代码
            sleep(1.5)
            print(f"[Robot] 已到达坐标 {world_coords}。")
            return True
        else:
            print(f"!!! [Robot] 错误: 未知的目标位置 '{location_name}'")
            return False

    def capture_and_save_image(self) -> str:
        print("[Robot] 正在拍照...")
        # TODO: 实现调用相机拍照并保存的代码
        sleep(0.5)
        image_path = "latest_capture.jpg"
        print(f"[Robot] 图片已保存至 '{image_path}'。")
        return image_path

    def perform_pick(self):
        print("[Robot] 正在执行抓取动作 (下降 -> 闭合夹爪)...")
        # TODO: 实现完整的抓取序列代码
        sleep(1.0)
        print("[Robot] 抓取完成。")

    def perform_drop(self):
        print("[Robot] 正在执行放置动作 (张开夹爪)...")
        # TODO: 实现放置序列代码
        sleep(0.5)
        print("[Robot] 放置完成。")
        
    def shutdown(self):
        print("[Robot] RobotController 已关闭。")

# ---------------------------------------------------------------------------
# 【核心】整合了校准功能的任务编排Agent
# ---------------------------------------------------------------------------
class TaskOrchestrator:
    def __init__(self):
        self.robot = RobotController()
        self.vlm = VLMDetector()  # 实例化VLM模块
        self.calibrator = CameraCalibrator(**CALIBRATION_CONFIG) # 实例化校准模块
        self.memory = {}
        self.last_image_path = None
        print("[Orchestrator] Agent 已初始化，所有模块已加载。")

    def convert_bbox_to_world_coords(self, bbox: list) -> list:
        """【已实现】使用校准器将归一化的bbox中心点转换为世界三维坐标。"""
        xmin, ymin, xmax, ymax = bbox
        norm_cx = (xmin + xmax) / 2.0
        norm_cy = (ymin + ymax) / 2.0
        pixel_cx = norm_cx * self.calibrator.img_w
        pixel_cy = norm_cy * self.calibrator.img_h
        
        print(f"[Orchestrator] Bbox中心点: 像素坐标 ({pixel_cx:.2f}, {pixel_cy:.2f})")
        world_x, world_y = self.calibrator.convert(pixel_cx, pixel_cy)
        world_coords = [world_x, world_y, TARGET_Z_HEIGHT]
        
        print(f"[Orchestrator] 转换完成: 世界坐标 ({world_x:.4f}, {world_y:.4f}, {TARGET_Z_HEIGHT})")
        return world_coords

    def execute_command(self, user_command: str):
        print("="*40)
        print(f"执行新指令: '{user_command}'")
        print("="*40)

        plan = plan_robot_tasks(user_command)
        if not plan:
            print("[Orchestrator] 无法生成任务计划，任务中止。")
            return

        for i, task in enumerate(plan):
            task_type = task["task_type"]
            params = task["parameters"]
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
                    location_name = params['location_name']
                    success = self.robot.move_to_location(location_name, self.memory)
                    if not success:
                        raise Exception(f"机器人移动到 '{location_name}' 失败。")
                    
                    if location_name == "observation_pose":
                        self.last_image_path = self.robot.capture_and_save_image()
                        
                elif task_type == "pick":
                    self.robot.perform_pick()
                    
                elif task_type == "drop":
                    self.robot.perform_drop()

                else:
                    print(f"警告：未知的任务类型 '{task_type}'，已跳过。")

            except Exception as e:
                print(f"!!! 错误：在执行第 {i+1} 步 ({task_type}) 时发生: {e}")
                print("!!! 任务计划中止。")
                self.robot.shutdown()
                return
        
        print("\n[Orchestrator] 任务计划已成功完成！")
        self.robot.shutdown()

# ---------------------------------------------------------------------------
# 主程序入口
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        agent = TaskOrchestrator()
        
        command = "把那个卷尺拿给我"
        print(f"你好，正在处理指令: '{command}'")
        
        if command:
            agent.execute_command(command)
    
    except Exception as e:
        print(f"程序启动时发生严重错误: {e}")