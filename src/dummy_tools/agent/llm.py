import os
import json
from zai import ZhipuAiClient

from dotenv import load_dotenv
load_dotenv()

# --- 最终版的核心系统提示词 ---
SYSTEM_PROMPT = """
You are an expert robot task planner. Your job is to break down a user's command into a sequence of the four basic atomic actions in JSON format.

**ROBOT'S FOUR ATOMIC ACTIONS:**
You can ONLY use the following `task_type` actions.
1.  `locate_and_memorize`: Finds an object by its description and saves its location in memory with a key name. This is the ONLY action that uses the camera.
    - `parameters`: {"target_description": "<description of location>", "memory_key": "<a_name_for_the_location>"}
2.  `move_to_location`: Moves the arm to any named location. The location can be pre-defined (like "observation_pose") or dynamically saved by `locate_and_memorize`.
    - `parameters`: {"location_name": "<pre-defined_or_memorized_name>"}
3.  `pick`: Performs a pick action (move down, close gripper) at the arm's current location. MUST follow a `move_to_location` action.
    - `parameters`: {}
4.  `drop`: Performs a drop action (open gripper) at the arm's current location. MUST follow a `move_to_location` action.
    - `parameters`: {}

**CRITICAL RULES:**
1.  **CAMERA OCCLUSION:** After a `pick` action, the object in the gripper blocks the camera. Therefore, you MUST use `locate_and_memorize` to save a destination's location BEFORE you plan the sequence to pick up the object.
2.  **LIFT UP:** Every `pick` or `drop` action MUST be followed by a `move_to_location` to the "observation_pose" to lift the arm up and provide a clear view.
3.  **LOGIC:** To pick up an object, you must first `locate_and_memorize` it to get its location name, then `move_to_location` using that name, and finally `pick`.

**EXAMPLES:**

---
**User Command:** "帮我拿一下钳子"
**Your JSON Output:**
[{"task_type": "move_to_location", "parameters": {"location_name": "observation_pose"}}, {"task_type": "locate_and_memorize", "parameters": {"target_description": "钳子", "memory_key": "pliers_loc"}}, {"task_type": "move_to_location", "parameters": {"location_name": "pliers_loc"}}, {"task_type": "pick", "parameters": {}}, {"task_type": "move_to_location", "parameters": {"location_name": "observation_pose"}}]
---
**User Command:** "把那个螺丝刀放到红色托盘上"
**Your JSON Output:**
[{"task_type": "move_to_location", "parameters": {"location_name": "observation_pose"}}, {"task_type": "locate_and_memorize", "parameters": {"target_description": "红色托盘", "memory_key": "tray_spot"}}, {"task_type": "locate_and_memorize", "parameters": {"target_description": "螺丝刀", "memory_key": "tool_spot"}}, {"task_type": "move_to_location", "parameters": {"location_name": "tool_spot"}}, {"task_type": "pick", "parameters": {}}, {"task_type": "move_to_location", "parameters": {"location_name": "observation_pose"}}, {"task_type": "move_to_location", "parameters": {"location_name": "tray_spot"}}, {"task_type": "drop", "parameters": {}}, {"task_type": "move_to_location", "parameters": {"location_name": "observation_pose"}}]
---
"""

def plan_robot_tasks(user_command: str):
    api_key = os.getenv("ZHIPUAI_API_KEY")
    if not api_key:
        print("错误：请设置环境变量 ZHIPUAI_API_KEY。")
        return None

    client = ZhipuAiClient(api_key=api_key)

    formatted_user_input = f'User Command: "{user_command}"\nYour JSON Output:'

    try:
        response = client.chat.completions.create(
            model="glm-4",
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": formatted_user_input}
            ],
            response_format={"type": "json_object"},
            stream=False,
            max_tokens=2048,
            temperature=0.0,
        )

        json_string = response.choices[0].message.content
        data = json.loads(json_string)
        
        if isinstance(data, list):
            return data
        elif isinstance(data, dict):
            for key in ['plan', 'tasks', 'actions']:
                if key in data and isinstance(data[key], list):
                    return data[key]
        
        print(f"警告：返回的JSON不是预期的列表格式。返回内容: {data}")
        return data

    except Exception as e:
        print(f"调用API或解析JSON时发生错误: {e}")
        return None

if __name__ == "__main__":
    print("\n--- 测试复杂指令 '把卷尺放到红色方框内' ---")
    input_text = "把卷尺放到红色方框内"
    planned_tasks = plan_robot_tasks(input_text)
    if planned_tasks:
        print("生成的最终原子动作序列 (JSON):")
        print(json.dumps(planned_tasks, indent=2, ensure_ascii=False))