#!/usr/bin/env python3
import subprocess
import signal
import sys
import os
import time

WS = "/home/li/ros2_dummy_arm"
SETUP = f"{WS}/install/setup.bash"

# Helper to run a command in a login shell with workspace sourced
def run_in_shell(cmd: str, cwd: str = WS, background: bool = True):
    full_cmd = f'bash -lc "source {SETUP} >/dev/null 2>&1; cd {cwd}; {cmd}"'
    if background:
        return subprocess.Popen(full_cmd, shell=True)
    else:
        return subprocess.run(full_cmd, shell=True, check=False)

bg_procs = []

def terminate_background():
    for p in bg_procs:
        try:
            if p.poll() is None:
                p.terminate()
        except Exception:
            pass
    # give them a moment, then kill if needed
    time.sleep(1.0)
    for p in bg_procs:
        try:
            if p.poll() is None:
                p.kill()
        except Exception:
            pass

def main():
    print("[start_all] 启动顺序: 控制器 → MoveIt(含RViz) → 添加障碍物 → 规划器")

    # 1) 控制器（后台）
    print("[start_all] 启动: ros2 run dummy_controller dummy_arm_controller (后台)")
    p_ctrl = run_in_shell("ros2 run dummy_controller dummy_arm_controller", background=True)
    bg_procs.append(p_ctrl)

    # 2) MoveIt与RViz（后台）
    # 当前launch未提供关闭RViz的参数，如需无界面运行需修改launch；此处按现有launch启动
    print("[start_all] 启动: ros2 launch dummy_moveit_config demo_real_arm.launch.py (后台)")
    p_launch = run_in_shell("ros2 launch dummy_moveit_config demo_real_arm.launch.py", background=True)
    bg_procs.append(p_launch)

    # 等待服务启动一小段时间
    time.sleep(3.0)

    # 3) 添加障碍物（一次性前台，自动结束）
    print("[start_all] 运行: python3 add_obstacles.py (前台一次性)")
    run_in_shell("python3 add_obstacles.py", background=False)

    # 4) 运行规划器（前台，保持终端停留）
    print("[start_all] 运行: python3 moveit_rviz_planner.py (前台)")
    try:
        run_in_shell("python3 moveit_rviz_planner.py", background=False)
    except KeyboardInterrupt:
        print("\n[start_all] 接收到中断信号，准备退出...")
    finally:
        print("[start_all] 清理后台进程...")
        terminate_background()
        print("[start_all] 已退出")

if __name__ == "__main__":
    # 传播Ctrl-C到子进程的简单处理
    signal.signal(signal.SIGINT, signal.default_int_handler)
    main() 