#!/bin/bash
# FastDDS共享内存问题修复脚本

echo "🔧 修复FastDDS共享内存传输问题..."

# 方法1: 禁用共享内存传输，使用UDP
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/li/ros2_dummy_arm/fastdds_no_shm.xml

echo "✅ 已设置FastDDS使用UDP传输而非共享内存"
echo "📝 当前配置:"
echo "   RMW_FASTRTPS_USE_QOS_FROM_XML=$RMW_FASTRTPS_USE_QOS_FROM_XML"
echo "   FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"

# 清理可能存在的锁定文件
echo "🧹 清理共享内存锁定文件..."
sudo rm -f /dev/shm/fastrtps_* 2>/dev/null || true

echo "🎯 现在可以启动dummy_arm_controller了:"
echo "   ros2 run dummy_controller dummy_arm_controller" 