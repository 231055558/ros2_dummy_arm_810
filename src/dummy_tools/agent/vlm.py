# vlm.py

import os
import json
import base64
import mimetypes
from pathlib import Path
from zai import ZhipuAiClient
from typing import List, Optional

from dotenv import load_dotenv
load_dotenv()

# --- 提示词保持不变 ---
VLM_SYSTEM_PROMPT = """# OBJECT DETECTION TASK

**Task:**
Identify the object specified by "target_description" in the image and return its bounding box. The description can be English or Chinese.

**Output Format (MANDATORY):**
- Your entire output MUST be a single, valid JSON object. No other text, comments, or explanations.
- If the object is found, return a JSON array of 4 floats: `[xmin, ymin, xmax, ymax]`.
- If the object is NOT found, return the JSON value `null`.

**Rules:**
1. Coordinates MUST be normalized (0.0 to 1.0), with (0.0, 0.0) at the top-left.
2. If multiple objects match, return the bbox for the most prominent/central one. ONLY return one.

**Examples:**
- User Input: {"target_description": "the red block"}
- Your Output: [0.25, 0.25, 0.5, 0.5]

- User Input: {"target_description": "a hammer"}
- Your Output: null

- User Input: {"target_description": "那个蓝色的圆形"}
- Your Output: [0.625, 0.625, 0.875, 0.875]
"""

def _encode_image_to_base64(image_path: Path) -> Optional[str]:
    """Helper function to encode an image file to a Base64 data URL string."""
    if not image_path.exists():
        print(f"Error: File not found at {image_path}")
        return None
    
    mime_type, _ = mimetypes.guess_type(image_path)
    if mime_type is None:
        mime_type = "application/octet-stream"

    with open(image_path, "rb") as image_file:
        base64_encoded_data = base64.b64encode(image_file.read()).decode('utf-8')

    return f"data:{mime_type};base64,{base64_encoded_data}"

class VLMDetector:
    """A class to handle all interactions with the Vision Language Model."""
    def __init__(self):
        api_key = os.getenv("ZHIPUAI_API_KEY")
        if not api_key:
            raise ValueError("错误：请设置环境变量 ZHIPUAI_API_KEY。")
        self.client = ZhipuAiClient(api_key=api_key)
        print("[VLM] VLMDetector 初始化完成。")

    def get_object_bbox(self, target_description: str, image_path: Path) -> Optional[List[float]]:
        """
        Uses the VLM to find an object and returns its normalized bounding box.
        """
        base64_image_url = _encode_image_to_base64(image_path)
        if not base64_image_url:
            return None

        user_request_json = json.dumps({"target_description": target_description}, ensure_ascii=False)
        full_text_prompt = f"{VLM_SYSTEM_PROMPT}\n\n**User Input:**\n{user_request_json}\n\n**Your Output:**"

        try:
            response = self.client.chat.completions.create(
                model="glm-4v", 
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": full_text_prompt},
                            {"type": "image_url", "image_url": {"url": base64_image_url}}
                        ]
                    }
                ],
                max_tokens=100,
                temperature=0.0,
            )

            result_text = response.choices[0].message.content.strip()

            if result_text.lower() == 'null':
                print(f"Info: VLM reported that '{target_description}' was not found.")
                return None
            
            bbox = json.loads(result_text)
            
            if isinstance(bbox, list) and len(bbox) == 4 and all(isinstance(n, (int, float)) for n in bbox):
                return [float(n) for n in bbox]
            else:
                print(f"Error: VLM returned an invalid format: {result_text}")
                return None

        except Exception as e:
            print(f"An error occurred during VLM API call or parsing: {e}")
            return None

# -------------------- MAIN LOGIC (Updated to use the class) --------------------
if __name__ == "__main__":
    try:
        # 1. 创建 VLMDetector 的实例
        vlm_detector = VLMDetector()
        
        test_image_path = Path("test_workspace.jpg")
        
        # 创建测试图片... (代码不变)
        try:
            from PIL import Image, ImageDraw
            img = Image.new('RGB', (400, 400), 'white')
            draw = ImageDraw.Draw(img)
            draw.rectangle([100, 100, 200, 200], fill='red')
            draw.ellipse([250, 250, 350, 350], fill='blue')
            img.save(test_image_path)
            print(f"Created a test image at '{test_image_path}'")
        except ImportError:
            print("Warning: Pillow not found. Please create 'test_workspace.jpg' manually.")

        # 2. 调用实例的方法来进行测试
        print("\n--- Test 1: Find '红色的方块' ---")
        bbox_1 = vlm_detector.get_object_bbox("红色的方块", test_image_path)
        print(f"Result: {bbox_1}")

        print("\n--- Test 2: Find '那个蓝色的圆形' ---")
        bbox_2 = vlm_detector.get_object_bbox("那个蓝色的圆形", test_image_path)
        print(f"Result: {bbox_2}")

    except ValueError as e:
        print(e) # 捕获API Key未设置的错误