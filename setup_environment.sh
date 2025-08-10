#!/bin/bash
# ROS2环境设置脚本 - 解决FastDDS共享内存问题

echo "🔧 设置ROS2环境并修复FastDDS问题..."

# 设置ROS2工作空间
source /opt/ros/humble/setup.bash
source install/setup.bash

# 修复FastDDS共享内存传输问题
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 清理可能的临时文件
rm -rf /tmp/fastrtps_* /dev/shm/fastrtps_* 2>/dev/null || true

# 重启ROS2守护进程以应用新设置
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start

echo "✅ 环境设置完成！"
echo "📝 已应用的修复:"
echo "   - 禁用FastDDS共享内存传输"
echo "   - 使用UDP传输协议"
echo "   - 清理临时文件"
echo "   - 重启ROS2守护进程"
echo ""
echo "🎯 现在可以安全运行:"
echo "   ros2 run dummy_controller dummy_arm_controller"
echo "   ros2 launch dummy_moveit_config demo_real_arm_headless.launch.py"
echo "   python3 add_obstacles.py"
echo "   python3 moveit_rviz_planner.py" 