# calibration_checker.py

import sys
import math
from typing import List

# ---------------------------------------------------------------------------
# 【配置区】请确保此处的配置与你的 agent.py 完全一致！
# ---------------------------------------------------------------------------
CALIBRATION_CONFIG = {
    "image_width": 1920,
    "image_height": 1080,
    "point1_px": [81, 1018],
    "point1_world": [0.1830301444979457, -0.19055072363908145],
    "point2_px": [1846, 92],
    "point2_world": [-0.12583986403766642, -0.35024648245505974]
}
# 镜像问题开关
SWAP_CALIBRATION_X_POINTS = False
SWAP_CALIBRATION_Y_POINTS = False

# 目标Z高度
TARGET_Z_HEIGHT = 0.12

# ---------------------------------------------------------------------------
# 拷贝自 agent.py 的校准逻辑
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

        if SWAP_CALIBRATION_X_POINTS:
            wx1, wx2 = wx2, wx1
        if SWAP_CALIBRATION_Y_POINTS:
            wy1, wy2 = wy2, wy1

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
# 主测试逻辑
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("="*50)
    print("      相机-世界坐标系校准检查工具")
    print("="*50)
    print("说明:")
    print("1. 请在另一个窗口打开实时相机画面。")
    print("2. 在本窗口输入你从相机画面中选取的像素点坐标。")
    print("3. 本程序会计算出对应的机器人世界坐标。")
    print("4. 请使用你的机器人控制器，手动控制机械臂移动到该世界坐标。")
    print("5. 观察相机画面，验证机械臂末端是否到达了你选择的像素点。")
    print("\n输入 'q' 或 'exit' 退出程序。\n")

    try:
        calibrator = CameraCalibrator(**CALIBRATION_CONFIG)
    except ValueError as e:
        print(f"初始化失败: {e}")
        sys.exit(1)

    while True:
        try:
            raw_input = input("请输入像素坐标 (格式: x y, 例如: 960 540): ")
            
            if raw_input.lower() in ['q', 'quit', 'exit']:
                break
            
            px_str, py_str = raw_input.split()
            px = int(px_str)
            py = int(py_str)

            # --- 执行计算 ---
            world_x, world_y = calibrator.convert(px, py)
            final_world_coords = [world_x, world_y, TARGET_Z_HEIGHT]
            # ---

            print("-" * 30)
            print(f"  输入像素坐标 (px, py): ({px}, {py})")
            print(f"  ✅ 计算出的世界坐标 (X, Y, Z): [{final_world_coords[0]:.4f}, {final_world_coords[1]:.4f}, {final_world_coords[2]:.2f}]")
            print("-" * 30)
            print(f"  下一步 -> 请手动控制机械臂移动到上述世界坐标，观察是否对准了像素点 ({px}, {py})。")
            print("\n")

        except (ValueError, IndexError):
            print("输入格式错误，请输入两个由空格隔开的数字。\n")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"发生未知错误: {e}")
            break
    
    print("校准检查程序已退出。")
