# vlm_bbox_checker.py

import os
from pathlib import Path
import cv2
import numpy as np
import pyrealsense2 as rs
from PIL import Image # 用于自动打开图片

# 导入你写好的VLM模块
from vlm import VLMDetector 

# --- 配置 ---
# 与你的agent.py保持一致，以便正确初始化相机
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
CAPTURE_FILENAME = Path("vlm_test_capture.jpg")
RESULT_FILENAME = Path("vlm_result_annotated.jpg")

def capture_image_realsense(filename: Path) -> bool:
    """使用RealSense相机拍照并保存。"""
    print("正在初始化RealSense相机...")
    pipeline = None
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, 30)
        pipeline.start(config)
        
        print("等待相机稳定...")
        for _ in range(15): # 等待15帧让自动曝光和白平衡稳定
            pipeline.wait_for_frames()
        
        print("正在拍照...")
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("无法从RealSense获取彩色图像帧。")
        
        # 将图像数据转换为OpenCV格式
        color_image = np.asanyarray(color_frame.get_data())
        
        # 保存图片
        cv2.imwrite(str(filename), color_image)
        print(f"图片已成功保存至 '{filename}'。")
        return True
    
    except Exception as e:
        print(f"!!! 拍照失败: {e}")
        return False
    finally:
        if pipeline:
            pipeline.stop()
            print("相机已关闭。")

def draw_bbox_on_image(image_path: Path, bbox: list, output_path: Path):
    """在图片上绘制Bbox和中心点。"""
    if not bbox or len(bbox) != 4:
        print("无效的Bbox，无法绘制。")
        return
        
    print(f"正在绘制Bbox {bbox} 到图片上...")
    
    # 读取图片
    image = cv2.imread(str(image_path))
    h, w, _ = image.shape
    
    # 将归一化的bbox坐标转换为像素坐标
    xmin_norm, ymin_norm, xmax_norm, ymax_norm = bbox
    xmin = int(xmin_norm * w)
    ymin = int(ymin_norm * h)
    xmax = int(xmax_norm * w)
    ymax = int(ymax_norm * h)
    
    # 计算中心点
    cx = int((xmin + xmax) / 2)
    cy = int((ymin + ymax) / 2)
    
    # 绘制矩形框 (绿色，厚度为3)
    cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 3)
    
    # 绘制中心点 (红色圆点，半径为10)
    cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
    
    # 在框的上方添加标签
    label = f"Center: ({cx}px, {cy}px)"
    (label_width, label_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 2)
    cv2.rectangle(image, (xmin, ymin - label_height - baseline), (xmin + label_width, ymin), (0, 255, 0), -1)
    cv2.putText(image, label, (xmin, ymin - baseline), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 2)

    # 保存标注后的图片
    cv2.imwrite(str(output_path), image)
    print(f"标注结果已保存至 '{output_path}'。")

# -------------------- 主测试逻辑 --------------------
if __name__ == "__main__":
    try:
        # 1. 初始化VLM检测器
        vlm_detector = VLMDetector()
        
        # 2. 拍照
        if not capture_image_realsense(CAPTURE_FILENAME):
            print("无法继续测试，因为拍照失败。")
            exit()
            
        # 3. 获取用户输入
        target_description = input("\n请输入要查找的物体描述 (例如: 红色的螺丝刀): ")
        if not target_description:
            print("输入为空，退出测试。")
            exit()

        # 4. 调用VLM获取Bbox
        print(f"正在使用VLM查找 '{target_description}'...")
        bbox_result = vlm_detector.get_object_bbox(target_description, CAPTURE_FILENAME)
        
        # 5. 处理结果
        if bbox_result:
            print(f"✅ VLM成功返回Bbox: {bbox_result}")
            # 6. 绘制结果并保存
            draw_bbox_on_image(CAPTURE_FILENAME, bbox_result, RESULT_FILENAME)
            
            # 7. 自动打开结果图片进行检查
            try:
                img_to_show = Image.open(RESULT_FILENAME)
                img_to_show.show()
            except Exception as e:
                print(f"\n无法自动打开图片。请手动查看 '{RESULT_FILENAME}'。错误: {e}")

        else:
            print("❌ VLM未能找到该物体或返回了无效结果。")

    except ValueError as e:
        print(f"初始化错误: {e}")
    except Exception as e:
        print(f"程序发生未知错误: {e}")