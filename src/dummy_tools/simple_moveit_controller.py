#!/usr/bin/env python3
"""
Simple MoveIt 控制器
- 使能/去使能机械臂
- 按6关节角度(度/弧度)规划并执行
- 获取当前末端6D位姿(平移XYZ[m] + RPY[rad])
- 按输入6D笛卡尔位姿规划并执行
"""

import math
import sys
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import SetBool
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFK, GetPositionIK


class SimpleMoveItController(Node):
    def __init__(self):
        super().__init__('simple_moveit_controller')

        # 组名/基座/末端链接名需与 MoveIt 配置一致
        self.group_name = 'dummy_arm'
        self.base_frame = 'base_link'
        self.ee_link = 'link6_1_1'
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        # 若模型中还包含夹爪关节，但没有 joint_states 发布，可在此补齐默认值
        self.extra_passive_joints = ['joint7', 'joint8']
        self.extra_passive_default = 0.0

        # 服务/Action 客户端
        self.robot_enable_service = self.create_client(SetBool, 'dummy_arm/enable')
        self.gripper_enable_service = self.create_client(SetBool, 'dummy_arm/gripper_enable')
        self.gripper_open_service = self.create_client(SetBool, 'dummy_arm/gripper_open')
        self.gripper_close_service = self.create_client(SetBool, 'dummy_arm/gripper_close')
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # 订阅关节状态
        self.current_joint_positions: Optional[List[float]] = None
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10
        )

        self.get_logger().info('🧩 SimpleMoveItController 已启动')

    # ---------------------------- 基础回调与等待 ----------------------------
    def _joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])

    def wait_ready(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info('⏳ 等待 MoveGroup/使能/FK/IK/夹爪 服务...')
        if not self.robot_enable_service.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ 机械臂使能服务不可用')
            return False
        if not self.gripper_enable_service.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ 夹爪使能服务不可用')
            return False
        if not self.gripper_open_service.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ 夹爪打开服务不可用')
            return False
        if not self.gripper_close_service.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ 夹爪关闭服务不可用')
            return False
        if not self.move_group_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('❌ MoveGroup Action 不可用')
            return False
        if not self.fk_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ FK 服务/compute_fk 不可用')
            return False
        if not self.ik_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('❌ IK 服务/compute_ik 不可用')
            return False
        self.get_logger().info('✅ 所有服务就绪')
        return True

    # ---------------------------- 公共辅助 ----------------------------
    def _compose_robot_state(self) -> JointState:
        js = JointState()
        js.header = Header()
        # 基本6轴
        base_names = list(self.joint_names)
        base_pos = self.current_joint_positions or [0.0] * 6
        # 追加被动/夹爪关节，填默认值，避免 MoveIt 报状态不完整
        extra_names = list(self.extra_passive_joints)
        extra_pos = [self.extra_passive_default] * len(extra_names)
        js.name = base_names + extra_names
        js.position = list(base_pos) + extra_pos
        return js

    # ---------------------------- 1) 机械臂使能 ----------------------------
    def enable_robot(self, enable: bool) -> bool:
        try:
            req = SetBool.Request()
            req.data = bool(enable)
            action = '使能' if enable else '去使能'
            self.get_logger().info(f'🟢 请求{action}机械臂...')
            fut = self.robot_enable_service.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is None:
                self.get_logger().error(f'❌ 机械臂{action}超时')
                return False
            resp = fut.result()
            if not resp.success:
                self.get_logger().error(f'❌ 机械臂{action}失败: {resp.message}')
                return False
            self.get_logger().info(f'✅ 机械臂{action}成功')
            # 避免夹爪联动闭合
            if enable:
                try:
                    dis_req = SetBool.Request(); dis_req.data = False
                    gfut = self.gripper_enable_service.call_async(dis_req)
                    rclpy.spin_until_future_complete(self, gfut, timeout_sec=3.0)
                except Exception:
                    pass
            return True
        except Exception as e:
            self.get_logger().error(f'❌ 机械臂使能异常: {e}')
            return False

    def _build_joint_goal(self, joints_rad: List[float]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group_name
        # 起始状态
        req.start_state.joint_state = self._compose_robot_state()
        # 目标约束
        constraints = []
        for name, pos in zip(self.joint_names, joints_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.append(jc)
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = constraints
        req.goal_constraints = [goal_constraints]
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10
        return goal

    def _rpy_to_quaternion(self, rx: float, ry: float, rz: float) -> Tuple[float, float, float, float]:
        # RPY->Quaternion (XYZ fixed)
        cy = math.cos(rz * 0.5)
        sy = math.sin(rz * 0.5)
        cp = math.cos(ry * 0.5)
        sp = math.sin(ry * 0.5)
        cr = math.cos(rx * 0.5)
        sr = math.sin(rx * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

    def _quaternion_to_rpy(self, qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
        # Quaternion -> RPY (XYZ fixed)
        # 参考标准转换
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (roll, pitch, yaw)

    # ---------------------------- 2) 关节运动 ----------------------------
    def move_to_joint_positions(self, joints: List[float], unit: str = 'deg') -> bool:
        if len(joints) != 6:
            self.get_logger().error('关节数量必须为6')
            return False
        joints_rad = [math.radians(j) for j in joints] if unit.lower() == 'deg' else [float(j) for j in joints]
        goal = self._build_joint_goal(joints_rad)
        self.get_logger().info(f'📤 发送关节目标: {joints} ({unit})')
        fut = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error('❌ 发送关节目标超时')
            return False
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('❌ 关节目标被拒绝')
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=60.0)
        if rfut.result() is None:
            self.get_logger().error('❌ 执行超时')
            return False
        res = rfut.result()
        ok = getattr(res.result.error_code, 'val', 0) == 1
        self.get_logger().info('🎉 关节运动成功' if ok else '❌ 关节运动失败')
        return ok

    # ---------------------------- 3) 获取末端位姿 ----------------------------
    def get_current_ee_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        if self.current_joint_positions is None:
            self.get_logger().warn('尚未收到关节状态')
            return None
        # 构造 FK 请求
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.ee_link]
        req.robot_state.joint_state = self._compose_robot_state()
        fut = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error('❌ FK 请求超时')
            return None
        resp = fut.result()
        if resp.error_code.val != 1 or len(resp.pose_stamped) == 0:
            self.get_logger().error(f'❌ FK 失败，错误码: {resp.error_code.val}')
            return None
        pose: PoseStamped = resp.pose_stamped[0]
        px = pose.pose.position.x
        py = pose.pose.position.y
        pz = pose.pose.position.z
        qx = pose.pose.orientation.x
        qy = pose.pose.orientation.y
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w
        rx, ry, rz = self._quaternion_to_rpy(qx, qy, qz, qw)
        return (px, py, pz, rx, ry, rz)

    # ---------------------------- 4) 笛卡尔位姿运动 ----------------------------
    def move_to_cartesian_pose(self, x: float, y: float, z: float, rx: float, ry: float, rz: float, base_frame: str = 'base_link') -> bool:
        # 使用 IK 将位姿转换为关节角，然后调用关节运动
        qx, qy, qz, qw = self._rpy_to_quaternion(rx, ry, rz)
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.joint_state = self._compose_robot_state()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped.header.frame_id = base_frame
        req.ik_request.pose_stamped.pose.position.x = float(x)
        req.ik_request.pose_stamped.pose.position.y = float(y)
        req.ik_request.pose_stamped.pose.position.z = float(z)
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw
        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error('❌ IK 请求超时')
            return False
        resp = fut.result()
        if resp.error_code.val != 1:
            self.get_logger().error(f'❌ IK 求解失败，错误码: {resp.error_code.val}')
            return False
        joints_rad = list(resp.solution.joint_state.position[:6])
        # 规划执行
        return self.move_to_joint_positions(joints_rad, unit='rad')

    # ---------------------------- 5) 夹爪控制 ----------------------------
    def control_gripper(self, close_gripper: bool = True) -> bool:
        try:
            req = SetBool.Request(); req.data = True
            action_name = '关闭' if close_gripper else '打开'
            client = self.gripper_close_service if close_gripper else self.gripper_open_service
            self.get_logger().info(f'🤏 {action_name}夹爪...')
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is None:
                self.get_logger().error(f'❌ 夹爪{action_name}请求超时')
                return False
            resp = fut.result()
            if resp.success:
                self.get_logger().info(f'✅ 夹爪{action_name}成功')
                return True
            self.get_logger().error(f'❌ 夹爪{action_name}失败: {resp.message}')
            return False
        except Exception as e:
            self.get_logger().error(f'❌ 夹爪控制异常: {e}')
            return False

    def clamp_on(self) -> bool:
        ok = self.set_gripper_enable(True)
        if not ok:
            return False
        return self.control_gripper(close_gripper=True)

    def clamp_off_disable(self) -> bool:
        ok = self.control_gripper(close_gripper=False)
        time.sleep(1.0)
        ok2 = self.set_gripper_enable(False)
        return ok and ok2

    # ---------------------------- 6) 夹爪使能 ----------------------------
    def set_gripper_enable(self, enable: bool) -> bool:
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


def main():
    rclpy.init()
    node = SimpleMoveItController()
    try:
        if not node.wait_ready():
            return
        print('\n指令示例:')
        print('  1) enable 1           -> 使能机械臂')
        print('  2) enable 0           -> 去使能机械臂')
        print('  3) joints 0 0 0 0 0 0 deg|rad')
        print('  4) ee                  -> 获取当前末端位姿[XYZ(m), RPY(rad)]')
        print('  5) pose X Y Z RX RY RZ -> 目标位姿[米, 弧度]')
        print('  6) c                   -> 夹爪使能并关闭夹爪 (clamp_on)')
        print('  7) x                   -> 打开夹爪并去使能 (clamp_off)')
        print('  0) exit')
        while rclpy.ok():
            try:
                cmd = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue
            if cmd == '0' or cmd.lower() in ['exit', 'quit']:
                break
            tokens = cmd.split()
            key = tokens[0]
            if key == 'enable' and len(tokens) >= 2:
                node.enable_robot(tokens[1] not in ['0', 'false', 'False'])
            elif key == 'joints' and (len(tokens) == 8 or len(tokens) == 7):
                unit = 'deg'
                if len(tokens) == 8:
                    unit = tokens[7]
                vals = list(map(float, tokens[1:7]))
                node.move_to_joint_positions(vals, unit)
            elif key == 'ee':
                pose = node.get_current_ee_pose()
                print('当前末端位姿:', pose)
            elif key == 'pose' and len(tokens) == 7:
                X, Y, Z, RX, RY, RZ = map(float, tokens[1:7])
                node.move_to_cartesian_pose(X, Y, Z, RX, RY, RZ)
            elif key in ['c', 'clamp_on']:
                node.clamp_on()
            elif key in ['x', 'clamp_off']:
                node.clamp_off_disable()
            else:
                print('未知指令')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 