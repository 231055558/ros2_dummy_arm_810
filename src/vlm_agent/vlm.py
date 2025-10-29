# vlm.py (最终正确版 - 增加了智能归一化防线)

import os
import json
import base64
import mimetypes
import re
from pathlib import Path
from zai import ZhipuAiClient
from typing import List, Optional

from dotenv import load_dotenv
load_dotenv()

def _encode_image_to_base64(image_path: Path) -> Optional[str]:
    """Helper function to encode an image file to a Base64 data URL string."""
    if not image_path.exists():
        print(f"Error: File not found at {image_path}")
        return None
    
    mime_type, _ = mimetypes.guess_type(image_path)
    if mime_type is None: mime_type = "application/octet-stream"
    with open(image_path, "rb") as image_file:
        base64_encoded_data = base64.b64encode(image_file.read()).decode('utf-8')
    return f"data:{mime_type};base64,{base64_encoded_data}"

class VLMDetector:
    """A class to handle all interactions with the Vision Language Model."""
    def __init__(self, image_width=1920, image_height=1080):
        api_key = os.getenv("ZHIPUAI_API_KEY")
        if not api_key: raise ValueError("错误：请设置环境变量 ZHIPUAI_API_KEY。")
        self.client = ZhipuAiClient(api_key=api_key)
        # 重新引入图像尺寸，因为我们的防线需要它
        self.image_width = image_width
        self.image_height = image_height
        print("[VLM] VLMDetector 初始化完成。")

    def get_object_bbox(self, target_description: str, image_path: Path) -> Optional[List[float]]:
        """
        Uses the VLM to find an object, and intelligently parses and normalizes
        the model's unpredictable output.
        """
        base64_image_url = _encode_image_to_base64(image_path)
        if not base64_image_url: return None

        # 提示词保持不变，继续“劝说”模型返回归一化坐标
        full_text_prompt = (
            f"Your task is to find the object described as '{target_description}' in the image. "
            "You MUST respond with its **normalized bounding box** using the exact format: "
            "<|begin_of_box|>[[xmin, ymin, xmax, ymax]]<|end_of_box|>. "
            "The coordinates MUST be **normalized floats** between 0.0 and 1.0, where (0.0, 0.0) is the top-left corner. "
            "**Do NOT output pixel coordinates.** "
            "For example, a perfect response for an object in the center would be: "
            "<|begin_of_box|>[[0.45, 0.45, 0.55, 0.55]]<|end_of_box|>."
        )

        try:
            response = self.client.chat.completions.create(
                model="glm-4.5v", # 建议使用更新、更强的模型以提高成功率
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": full_text_prompt},
                            {"type": "image_url", "image_url": {"url": base64_image_url}}
                        ]
                    }
                ],
                max_tokens=1024,
                temperature=0.0,
            )

            raw_response_text = response.choices[0].message.content.strip()
            print(f"[VLM] 模型原始返回:\n---\n{raw_response_text}\n---")

            # 使用最稳健的字符串查找来提取bbox内容
            start_token = "<|begin_of_box|>"
            end_token = "<|end_of_box|>"
            start_index = raw_response_text.find(start_token)
            end_index = raw_response_text.find(end_token, start_index)

            if start_index == -1 or end_index == -1:
                print(f"Info: 未能在模型返回中找到 Bbox 的开始/结束标记。")
                return None

            bbox_str = raw_response_text[start_index + len(start_token):end_index]
            bboxes = json.loads(bbox_str)
            
            if not isinstance(bboxes, list) or not bboxes: return None
            first_bbox = bboxes[0]
            if not isinstance(first_bbox, list) or len(first_bbox) != 4: return None

            # --- 【核心修改：智能归一化防线】 ---
            coords = [float(c) for c in first_bbox]
            
            # 检测返回的是像素坐标还是归一化坐标
            if any(c > 1.0 for c in coords):
                print("[VLM 防线] 检测到像素坐标，正在进行手动归一化...")
                x1, y1, x2, y2 = coords
                normalized_bbox = [
                    x1 / 1000,
                    y1 / 1000,
                    x2 / 1000,
                    y2 / 1000
                ]
            else:
                print("[VLM 防线] 检测到归一化坐标，直接使用。")
                normalized_bbox = coords
            
            # 最终的安全裁剪
            normalized_bbox = [max(0.0, min(1.0, c)) for c in normalized_bbox]
            
            return normalized_bbox

        except Exception as e:
            print(f"VLM API调用或解析时发生错误: {e}")
            return None

# -------------------- MAIN LOGIC (用于独立测试) --------------------
if __name__ == "__main__":
    try:
        # 在初始化时必须传入图像尺寸，以备防线使用
        vlm_detector = VLMDetector(image_width=1920, image_height=1080)
        
        test_image_path = Path("latest_capture.jpg")
        if not test_image_path.exists():
            print(f"错误: 测试图片 '{test_image_path}' 不存在。")
        else:
            description = input("请输入要查找的物体描述 (例如: 筹码): ")
            if description:
                bbox_result = vlm_detector.get_object_bbox(description, test_image_path)
                print(f"\n--- VLM 最终归一化结果 ---")
                if bbox_result:
                    print(f"✅ 成功! 归一化 Bbox: {[f'{c:.4f}' for c in bbox_result]}")
                else:
                    print("❌ 失败!")

    except ValueError as e:
        print(e)