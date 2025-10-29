#!/usr/bin/env python3
"""
MoveIt RViz自动规划执行器
通过RViz的MoveIt环境进行规划，规划成功后自动执行
使用RViz中的障碍物检查和安全规划
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time
from threading import Thread
import re
import os
import json

# MoveIt相关服务和消息
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
from moveit_msgs.msg import (
    MotionPlanRequest, 
    WorkspaceParameters, 
    Constraints,
    JointConstraint,
    RobotState,
    DisplayTrajectory
)
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_srvs.srv import SetBool


class RVizMoveItDemo(Node):
    def __init__(self):
        super().__init__('rviz_moveit_demo')
        
        # MoveIt规划服务客户端 - 这个会使用RViz中的障碍物
        self.plan_service = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # MoveGroup Action客户端 - 用于执行规划好的轨迹
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # 夹爪控制服务客户端
        self.gripper_open_service = self.create_client(SetBool, 'dummy_arm/gripper_open')
        self.gripper_close_service = self.create_client(SetBool, 'dummy_arm/gripper_close')
        
        # 新增：整机使能与夹爪使能服务客户端
        self.robot_enable_service = self.create_client(SetBool, 'dummy_arm/enable')
        self.gripper_enable_service = self.create_client(SetBool, 'dummy_arm/gripper_enable')
        
        # 发布规划结果到RViz显示
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory, 
            '/move_group/display_planned_path', 
            10
        )
        
        # 订阅当前关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # 目标定义改为动态从 JSON 文件读取/写入（更稳定）
        self.targets_json = '/home/li/ros2_dummy_arm/targets.json'
        # 若文件不存在，创建并提供一个可参考的 home 示例（单位：deg）
        try:
            if not os.path.exists(self.targets_json):
                with open(self.targets_json, 'w', encoding='utf-8') as f:
                    json.dump({
                        "home": {
                            "unit": "deg",
                            "joints": [0, 0, 0, 0, 0, 0]
                        }
                    }, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().warn(f"初始化targets.json失败: {e}")
        
        # 记录点存储（运行期内存，优先于文件目标）
        self.recorded_targets = {}
        
        self.get_logger().info('🎯 RViz MoveIt自动规划执行器已启动')

    def joint_state_callback(self, msg):
        """接收当前关节状态"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])

    def wait_for_services(self, timeout=10.0):
        """等待MoveIt服务连接"""
        self.get_logger().info('⏳ 等待MoveIt服务连接...')
        
        if not self.plan_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ MoveIt规划服务不可用')
            return False
            
        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('❌ MoveGroup Action服务不可用')
            return False
            
        if not self.gripper_open_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ 夹爪打开服务不可用')
            return False
            
        if not self.gripper_close_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ 夹爪关闭服务不可用')
            return False

        # 新增：等待使能相关服务
        if not self.robot_enable_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ 整机使能服务不可用')
            return False
        if not self.gripper_enable_service.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('❌ 夹爪使能服务不可用')
            return False
            
        self.get_logger().info('✅ MoveIt服务连接成功')
        return True

    def create_motion_plan_request(self, target_positions):
        """创建运动规划请求"""
        request = GetMotionPlan.Request()
        
        # 设置运动规划请求
        motion_plan_request = MotionPlanRequest()
        
        # 设置群组名称
        motion_plan_request.group_name = "dummy_arm"
        
        # 设置起始状态（当前状态）
        motion_plan_request.start_state.joint_state.header = Header()
        motion_plan_request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = self.current_joint_positions or [0.0] * 6

        # 设置目标关节约束
        joint_constraints = []
        for i, (name, position) in enumerate(zip(self.joint_names, target_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            joint_constraints.append(joint_constraint)
        
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints
        motion_plan_request.goal_constraints = [goal_constraints]
        
        # 设置工作空间参数
        motion_plan_request.workspace_parameters = WorkspaceParameters()
        motion_plan_request.workspace_parameters.header.frame_id = "base_link"
        motion_plan_request.workspace_parameters.min_corner.x = -2.0
        motion_plan_request.workspace_parameters.min_corner.y = -2.0 
        motion_plan_request.workspace_parameters.min_corner.z = -2.0
        motion_plan_request.workspace_parameters.max_corner.x = 2.0
        motion_plan_request.workspace_parameters.max_corner.y = 2.0
        motion_plan_request.workspace_parameters.max_corner.z = 2.0
        
        # 设置规划器参数
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.1
        motion_plan_request.max_acceleration_scaling_factor = 0.1
        
        request.motion_plan_request = motion_plan_request
        return request

    def create_move_group_goal(self, target_positions):
        """创建MoveGroup执行目标"""
        goal = MoveGroup.Goal()
        
        # 创建规划请求
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "dummy_arm"
        
        # 设置起始状态
        motion_plan_request.start_state.joint_state.header = Header()
        motion_plan_request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = self.current_joint_positions or [0.0] * 6

        # 设置目标关节约束
        joint_constraints = []
        for i, (name, position) in enumerate(zip(self.joint_names, target_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            joint_constraints.append(joint_constraint)
        
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints
        motion_plan_request.goal_constraints = [goal_constraints]
        
        # 设置规划器参数
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 10.0  # 增加规划时间
        motion_plan_request.max_velocity_scaling_factor = 0.3  # 提高速度
        motion_plan_request.max_acceleration_scaling_factor = 0.3  # 提高加速度
        
        goal.request = motion_plan_request
        goal.planning_options.plan_only = False  # 规划并执行
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10  # 增加重新规划次数
        
        return goal

    def _parse_degrees_from_line(self, line: str):
        """从一行中解析角度（度），支持含有°符号与空格。返回float或None"""
        try:
            # 允许格式: "joint1: 12.34°" 或 "joint1: 12.34"
            # 提取第一个可解析的数字
            match = re.search(r'([-+]?\d+(?:\.\d+)?)', line)
            if match:
                return float(match.group(1))
        except Exception:
            pass
        return None

    def _read_target_from_json(self, target_name: str):
        """从 JSON 文件读取目标，返回[rad]*6 或 None。支持{"unit":"deg|rad","joints":[..]}或直接数组([deg])。"""
        path = self.targets_json
        try:
            if not os.path.exists(path):
                return None
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"❌ 读取 {path} 失败: {e}")
            return None

        if target_name not in data:
            return None
        entry = data[target_name]
        try:
            # 支持两种格式
            if isinstance(entry, dict) and 'joints' in entry:
                unit = entry.get('unit', 'deg').lower()
                joints = entry['joints']
            elif isinstance(entry, list):
                unit = 'deg'
                joints = entry
            else:
                self.get_logger().error(f"❌ 目标{target_name}格式不正确")
                return None
            if len(joints) != 6:
                self.get_logger().error(f"❌ 目标{target_name}关节数量应为6")
                return None
            joints = [float(x) for x in joints]
            if unit == 'deg':
                return [math.radians(v) for v in joints]
            elif unit == 'rad':
                return joints
            else:
                self.get_logger().error(f"❌ 目标{target_name}未知单位: {unit}")
                return None
        except Exception as e:
            self.get_logger().error(f"❌ 解析目标{target_name}失败: {e}")
            return None

    def _save_target_to_json(self, target_name: str, joints_rad):
        """将目标写入 JSON 文件（以deg保存，保留2位小数）。"""
        path = self.targets_json
        try:
            data = {}
            if os.path.exists(path):
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            degs = [round(math.degrees(v), 2) for v in joints_rad]
            data[target_name] = {"unit": "deg", "joints": degs}
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f"❌ 写入 {path} 失败: {e}")
            return False

    def plan_and_execute_target(self, target_name):
        """规划并自动执行到目标位置(从target.txt动态读取)"""
        # 优先使用运行期记录点
        if target_name in self.recorded_targets:
            target_positions = self.recorded_targets[target_name]
        else:
            # 动态从 JSON 读取（不缓存，支持运行时修改）
            target_positions = self._read_target_from_json(target_name)
        if target_positions is None:
            return False

        joint_degrees = [math.degrees(j) for j in target_positions]
        self.get_logger().info(f'🎯 通过RViz规划并执行到{target_name}: {[f"{j:.1f}°" for j in joint_degrees]}')
        
        try:
            # 创建MoveGroup目标
            goal = self.create_move_group_goal(target_positions)
            
            # 发送目标到MoveGroup
            self.get_logger().info(f'📤 发送{target_name}目标到MoveGroup...')
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            
            if send_goal_future.result() is None:
                self.get_logger().error(f'❌ {target_name}目标发送超时')
                return False
                
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'❌ {target_name}目标被拒绝')
                return False
            
            self.get_logger().info(f'✅ {target_name}目标已接受，开始规划和执行...')
            
            # 等待执行完成 - 增加超时时间
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)
            
            if get_result_future.result() is None:
                self.get_logger().error(f'❌ {target_name}执行超时')
                return False
                
            result = get_result_future.result()
            
            if result.result.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f'🎉 {target_name}规划和执行成功完成！')
                return True
            else:
                error_code = result.result.error_code.val
                self.get_logger().error(f'❌ {target_name}执行失败，错误码: {error_code}')
                if error_code == -1:
                    self.get_logger().error('   可能原因：目标位置不可达或存在碰撞')
                elif error_code == -2:
                    self.get_logger().error('   可能原因：规划超时')
                elif error_code == -3:
                    self.get_logger().error('   可能原因：无效的机器人状态')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ {target_name}规划执行异常: {e}')
            import traceback
            traceback.print_exc()
            return False

    def plan_and_execute_positions(self, name: str, positions_rad):
        """使用给定的关节弧度列表规划并执行"""
        try:
            goal = self.create_move_group_goal(positions_rad)
            self.get_logger().info(f'📤 发送{name}目标到MoveGroup...')
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            if send_goal_future.result() is None:
                self.get_logger().error(f'❌ {name}目标发送超时')
                return False
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'❌ {name}目标被拒绝')
                return False
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)
            if get_result_future.result() is None:
                self.get_logger().error(f'❌ {name}执行超时')
                return False
            result = get_result_future.result()
            if result.result.error_code.val == 1:
                self.get_logger().info(f'🎉 {name}规划和执行成功完成！')
                return True
            else:
                error_code = result.result.error_code.val
                self.get_logger().error(f'❌ {name}执行失败，错误码: {error_code}')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ {name}规划执行异常: {e}')
            import traceback
            traceback.print_exc()
            return False

    def control_gripper(self, close_gripper=True):
        """控制夹爪开关"""
        try:
            request = SetBool.Request()
            request.data = True  # 服务只需要触发，数据内容不重要
            
            action_name = "关闭" if close_gripper else "打开"
            service_client = self.gripper_close_service if close_gripper else self.gripper_open_service
            
            self.get_logger().info(f'🤏 {action_name}夹爪...')
            
            future = service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None:
                self.get_logger().error(f'❌ 夹爪{action_name}请求超时')
                return False
                
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 夹爪{action_name}成功')
                return True
            else:
                self.get_logger().error(f'❌ 夹爪{action_name}失败: {response.message}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ 夹爪控制异常: {e}')
            return False

    def set_robot_enable(self, enable: bool):
        """整机使能/去使能"""
        try:
            request = SetBool.Request()
            request.data = bool(enable)
            action = '使能' if enable else '去使能'
            self.get_logger().info(f'🟢 请求{action}机械臂...')
            future = self.robot_enable_service.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                self.get_logger().error(f'❌ 机械臂{action}请求超时')
                return False
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'✅ 机械臂{action}成功')
                # 当机械臂被使能时，确保夹爪不会被联动使能/闭合
                if enable:
                    # 取消夹爪使能，并尝试保持夹爪为打开状态
                    try:
                        self.set_gripper_enable(False)
                        # 确保夹爪打开（某些底层在上电会默认闭合）
                        self.control_gripper(close_gripper=False)
                    except Exception as _:
                        pass
                return True
            self.get_logger().error(f'❌ 机械臂{action}失败: {resp.message}')
            return False
        except Exception as e:
            self.get_logger().error(f'❌ 机械臂使能控制异常: {e}')
            return False

    def set_gripper_enable(self, enable: bool):
        """夹爪使能/去使能"""
        try:
            request = SetBool.Request()
            request.data = bool(enable)
            action = '使能' if enable else '去使能'
            self.get_logger().info(f'🤏 请求{action}夹爪...')
            future = self.gripper_enable_service.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                self.get_logger().error(f'❌ 夹爪{action}请求超时')
                return False
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'✅ 夹爪{action}成功')
                return True
            self.get_logger().error(f'❌ 夹爪{action}失败: {resp.message}')
            return False
        except Exception as e:
            self.get_logger().error(f'❌ 夹爪使能控制异常: {e}')
            return False

    def get_current_joint_angles_degrees(self):
        """获取当前关节角度（度数）"""
        if self.current_joint_positions:
            return [math.degrees(j) for j in self.current_joint_positions]
        return None

    def display_current_status(self, step_name):
        """显示当前状态"""
        print("\n" + "="*60)
        print(f"📍 {step_name}")
        print("="*60)
        
        # 显示关节角度
        joint_angles = self.get_current_joint_angles_degrees()
        if joint_angles:
            print("📐 当前关节角度:")
            for i, angle in enumerate(joint_angles):
                print(f"  joint{i+1}: {angle:8.2f}°")
            # 追加：可直接复制到 target.txt 的块（名称占位符）
            print("\n可复制到 targets.json（示例）：")
            print("{")
            print(f"  \"current_pose\": {{ \"unit\": \"deg\", \"joints\": [{', '.join(f'{a:.2f}' for a in joint_angles)}] }}")
            print("}")
        else:
            print("⚠️  等待关节状态数据...")
        
        print("="*60)

    def save_current_pose(self, name: str):
        """保存当前位姿到记录表，并打印可复制到target.txt的格式"""
        if not self.current_joint_positions or len(self.current_joint_positions) < 6:
            print("⚠️  尚未获取到有效关节状态，无法保存")
            return False
        # 保存弧度
        self.recorded_targets[name] = list(self.current_joint_positions[:6])
        # 打印可复制格式（度）
        degs = [math.degrees(j) for j in self.current_joint_positions[:6]]
        print(f"\n已保存记录点: {name}")
        print("可复制到 targets.json（示例）：")
        print("{")
        print(f"  \"{name}\": {{ \"unit\": \"deg\", \"joints\": [{', '.join(f'{a:.2f}' for a in degs)}] }}")
        print("}")
        # 同时持久化到 JSON 文件
        self._save_target_to_json(name, self.recorded_targets[name])
        return True

    def run_add_obstacles(self):
        """调用 add_obstacles.py 增加障碍"""
        try:
            import subprocess
            print("⛳ 正在添加障碍物...")
            # 阻塞运行一次，完成后返回
            subprocess.run(['bash', '-lc', 'cd /home/li/ros2_dummy_arm; source install/setup.bash >/dev/null 2>&1; python3 add_obstacles.py'], check=False)
            print("✅ 障碍物添加完成")
        except Exception as e:
            print(f"❌ 添加障碍失败: {e}")

    def perform_sequence_demo(self):
        """执行完整的自动序列演示"""
        self.get_logger().info('🎭 开始执行RViz自动规划执行序列')
        print("\n🎯 运动序列: 目标2 → 目标3 → 目标1 → 目标4 → 重置位置")
        print("🤖 通过RViz MoveIt环境规划，规划成功后自动执行")
        print("🤏 特殊操作：目标2处关闭夹爪，目标4处打开夹爪")
        
        # 运动序列：2 → 3 → 1 → 4 → reset（名称从target.txt中读取）
        sequence = [
            ('target2', '目标2'),
            ('target3', '目标3'), 
            ('target1', '目标1'),
            ('target4', '目标4'),
            ('reset', '重置位置')
        ]
        
        for i, (target_key, target_name) in enumerate(sequence):
            step_num = i + 1
            total_steps = len(sequence)
            
            print(f"\n🔄 步骤 {step_num}/{total_steps}: 规划并执行到{target_name}")
            
            # 规划并执行到目标位置
            if not self.plan_and_execute_target(target_key):
                self.get_logger().error(f'❌ {target_name}失败，序列终止')
                return False
            
            # 显示当前状态
            self.display_current_status(f"已到达{target_name}")
            
            # 特殊操作：在目标2关闭夹爪，在目标4打开夹爪
            if target_key == 'target1':
                print("🤏 到达目标2，3秒后关闭夹爪...")
                time.sleep(3.0)
                self.control_gripper(close_gripper=True)  # 关闭夹爪
            elif target_key == 'target4':
                print("🤏 到达目标4，3秒后打开夹爪...")
                time.sleep(3.0)
                self.control_gripper(close_gripper=False)  # 打开夹爪
            
            # 如果不是最后一个位置，等待3秒
            if step_num < total_steps:
                if target_key not in ['target2', 'target4']:  # 目标2和4已经等待过3秒了
                    print(f"⏳ 在{target_name}停留3秒...")
                    for remaining in range(3, 0, -1):
                        print(f"  倒计时: {remaining}秒", end='\r')
                        time.sleep(1.0)
                    print("  继续下一个目标...     ")
                else:
                    print("  继续下一个目标...")
            else:
                print(f"🏁 序列完成，已到达{target_name}")
        
        self.get_logger().info('🎉 RViz自动规划执行序列完成')
        return True

    def show_menu(self):
        """显示菜单"""
        print("\n" + "="*70)
        print("🎯 RViz MoveIt自动规划执行器")
        print("="*70)
        print("💡 通过RViz MoveIt环境规划，规划成功后自动执行")
        print("🛡️  使用RViz中的障碍物进行安全检查")
        print("="*70)
        print("目标与模式:")
        print("  直接输入目标名（如 home）→ 从targets.json动态读取并执行（运行时可热更新）")
        print("  p. record    - 进入记录点模式：输入英文名立即保存当前位姿，'back'退出")
        print("  o. cruise    - 进入巡航模式：输入记录点名或targets.json目标名立即执行，'back'退出")
        print("  n. now       - 记录当前位姿并按格式输出(自动命名)")
        print()
        print("使能与夹爪:")
        print("  e. enable    - 机械臂使能 (不联动夹爪)")
        print("  d. disable   - 机械臂去使能")
        print("  c. clamp_on  - 夹爪使能并关闭夹爪")
        print("  x. clamp_off - 打开夹爪并取消夹爪使能")
        print()
        print("其他:")
        print("  a. add_obs   - 运行 add_obstacles.py 增加障碍")
        print("  7. current   - 显示当前状态（附JSON示例，便于加入targets.json）")
        print("  6. sequence  - 执行固定序列(target2→target3→target1→target4→reset)")
        print()
        print("  0. 退出")
        print("="*70)

    def show_current_state(self):
        """显示当前状态详情"""
        self.display_current_status("当前机械臂状态")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = RVizMoveItDemo()
        
        # 等待MoveIt服务连接
        if not demo.wait_for_services():
            print("❌ MoveIt服务连接失败，请确认已启动demo_real_arm.launch.py")
            return
        
        # 启动ROS spin线程
        ros_thread = Thread(target=rclpy.spin, args=(demo,))
        ros_thread.daemon = True
        ros_thread.start()
        
        # 等待获取初始关节状态
        print("⏳ 等待关节状态数据...")
        for _ in range(50):  # 等待5秒
            if demo.current_joint_positions:
                break
            time.sleep(0.1)
        
        # 显示初始状态
        demo.show_current_state()
        
        # 主菜单循环
        while True:
            demo.show_menu()
            user_input = input("\n请输入命令或目标名: ").strip()
            
            if user_input == '0' or user_input.lower() in ['exit', 'quit']:
                break
            elif user_input == '6' or user_input.lower() == 'sequence':
                demo.perform_sequence_demo()
            elif user_input == '7' or user_input.lower() == 'current':
                demo.show_current_state()
            elif user_input.lower() == 'e' or user_input.lower() == 'enable':
                demo.set_robot_enable(True)
            elif user_input.lower() == 'd' or user_input.lower() == 'disable':
                demo.set_robot_enable(False)
            elif user_input.lower() == 'c':
                # 夹爪使能并关闭夹爪
                demo.set_gripper_enable(True)
                demo.control_gripper(close_gripper=True)
            elif user_input.lower() == 'x':
                # 打开夹爪并去使能
                demo.control_gripper(close_gripper=False)
                time.sleep(1.0)
                demo.set_gripper_enable(False)
            elif user_input.lower() == 'a':
                demo.run_add_obstacles()
            elif user_input.lower() == 'p':
                # 记录点模式
                print("进入记录点模式：输入英文名保存当前位姿，输入 back 退出")
                while True:
                    name = input("记录名: ").strip()
                    if name.lower() == 'back':
                        break
                    if not name:
                        print("⚠️ 名称不能为空")
                        continue
                    demo.save_current_pose(name)
                print("退出记录点模式")
            elif user_input.lower() == 'o':
                # 巡航模式
                print("进入巡航模式：输入记录点名或 targets.json 中的目标名，输入 back 退出")
                while True:
                    name = input("目标名: ").strip()
                    if name.lower() == 'back':
                        break
                    if not name:
                        continue
                    if name in demo.recorded_targets:
                        demo.plan_and_execute_positions(name, demo.recorded_targets[name])
                    else:
                        if not demo.plan_and_execute_target(name):
                            print("❌ 未找到目标：", name)
                print("退出巡航模式")
            elif user_input.lower() == 'n':
                # 自动命名记录点
                base = 'saved'
                idx = 1
                while f'{base}_{idx}' in demo.recorded_targets:
                    idx += 1
                name = f'{base}_{idx}'
                demo.save_current_pose(name)
            else:
                # 将输入视为目标名，从targets.json/记录点动态读取执行
                if not demo.plan_and_execute_target(user_input):
                    print("❌ 未知命令或未找到目标：", user_input)
    
    except KeyboardInterrupt:
        print("\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 程序异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            demo.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 