# 引用相关仓库
### 由于本项目重新烧录了末端含自制夹爪的主控固件，因此引用仓库中的程序不全适用，但一般不会对机械臂产生损害，可以部分参考

> 木子改良版本的机械臂：https://gitee.com/switchpi/dummy.git [here](https://gitee.com/switchpi/dummy.git)!

> hata8210配置的moveit接入：https://github.com/hata8210/dummy_moveit_ws.git [here](https://github.com/hata8210/dummy_moveit_ws.git)!

# Dummy Robot Arm - ROS2 项目

基于 ROS2 Humble 的六自由度夹爪机械臂控制系统，集成 MoveIt 运动规划、视觉感知和智能代理功能。

## 项目概述

本项目提供了一套完整的机械臂控制解决方案，包括：
- 基于 MoveIt 的运动规划与执行
- USB 通信的底层控制器
- Intel RealSense 视觉集成
- VLM（视觉语言模型）智能代理
- 多种实用工具和示例程序

---

## 目录结构

### 📦 src 目录包含以下功能包：

#### 1. **dummy_controller** - 核心控制器包
机械臂的底层控制和通信模块。

**主要功能：**
- `dummy_arm_controller.py` - 机械臂 USB 控制器，负责与硬件通信
- `moveit_server.py` - MoveIt 服务器节点，提供运动规划服务
- `add_collision_object.py` - 碰撞对象管理，用于场景规划
- `depth_image_visualizer.py` - 深度图像可视化工具
- `dummy_cli_tool/` - 底层通信工具库（基于 fibre 协议）

#### 2. **dummy_moveit_config** - MoveIt 配置包
MoveIt 运动规划框架的配置文件集合。

**主要内容：**
- 运动学配置（kinematics）
- 规划器配置（OMPL）
- 控制器配置
- 启动文件（launch files）
  - `demo_real_arm.launch.py` - 真实机械臂启动文件
  - 其他仿真和调试启动文件

#### 3. **dummy-ros2_description** - 机器人描述包
机器人的 URDF/Xacro 模型定义。

**主要内容：**
- `urdf/` - 机器人 URDF 描述文件
- `meshes/` - 3D 模型文件
- `config/` - RViz 和其他配置
- `launch/` - 模型显示启动文件

#### 4. **dummy_server** - MoveIt Python 接口
提供 Python 版本的 MoveIt 控制接口。

**主要内容：**
- `pymoveit2/` - Python MoveIt2 库
- `examples/` - 使用示例

#### 5. **dummy_tools** - 实用工具集
各种便捷的控制和测试工具。

**主要工具：**
- `moveit_rviz_planner.py` - RViz 自动规划执行器，通过 RViz 进行可视化规划
- `simple_moveit_controller.py` - 简化的 MoveIt 控制器，支持关节和笛卡尔空间控制
- `add_obstacles.py` - 向规划场景添加障碍物
- `show_realsense_rgb.py` - RealSense 相机 RGB 图像显示

#### 6. **vlm_agent** - 视觉语言模型代理
集成视觉和语言模型的智能代理系统。

**主要模块：**
- `agent.py` / `agent_total.py` - 智能代理主程序
- `vlm.py` - 视觉语言模型接口
- `llm.py` - 大语言模型接口
- `stt.py` - 语音转文本模块
- `checker.py` - 状态检查器
- `vlm_bbox_show.py` - 目标检测边界框可视化

---

## 可用 Demo 示例

### 1. 基础运动控制
使用 `simple_moveit_controller.py` 进行关节空间和笛卡尔空间的运动控制。

### 2. RViz 可视化规划
使用 `moveit_rviz_planner.py` 在 RViz 中进行交互式规划和执行。

### 3. 障碍物规划
使用 `add_obstacles.py` 添加虚拟障碍物，测试避障规划功能。

### 4. 视觉感知
使用 `show_realsense_rgb.py` 查看 RealSense 相机图像。

### 5. 智能代理
运行 `vlm_agent` 中的代理程序，实现视觉引导的智能操作。

---

## 环境配置

### 系统要求
- **操作系统**: Ubuntu 22.04
- **ROS 版本**: ROS2 Humble

### 安装步骤

#### 1. 安装 ROS2 Humble

使用 fishros 一键安装脚本：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

#### 2. 安装依赖包

**ARM 交叉编译工具链：**
```bash
sudo apt update
sudo apt install gcc-arm-none-eabi
```

**ROS2 控制消息包：**
```bash
sudo apt install ros-humble-control-msgs
```

**MoveIt 运动规划框架：**
```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-servo
```

#### 3. 安装 Intel RealSense SDK（可选）

如果使用 RealSense 相机，需要安装以下内容：

**添加公钥：**
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

**添加仓库：**
```bash
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
```

**安装 SDK：**
```bash
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-dev
sudo apt install librealsense2-utils
```

#### 4. Python 环境配置

**安装 pip：**
```bash
sudo apt install python3-pip
```

**安装 pyusb：**
```bash
pip install pyusb
```

#### 5. USB 设备权限配置

**查找设备 VID 和 PID：**
```bash
lsusb
```

找到类似这样的输出：
```
Bus 002 Device 007: ID 1209:0d32 Generic ODrive Robotics ODrive v3
```

其中 `1209` 是 VID，`0d32` 是 PID。

**创建 udev 规则：**
```bash
sudo nano /etc/udev/rules.d/99-dummy-arm.rules
```

添加以下内容（替换为你的 VID 和 PID）：
```
# Rule for Dummy Arm Controller
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE="0666"
```

**使规则生效：**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 USB 设备。

---

## 编译与运行

### 编译工作空间

```bash
# 进入工作空间根目录（src 的父目录）
cd /path/to/dummy_demo

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译
colcon build

# 加载工作空间环境
source install/setup.bash
```

### 启动系统

**终端 1 - 启动 MoveIt 和 RViz：**
```bash
source install/setup.bash
ros2 launch dummy_moveit_config demo_real_arm.launch.py
```

**终端 2 - 启动机械臂控制器：**
```bash
source install/setup.bash
ros2 run dummy_controller dummy_arm_controller
```

**终端 3 - 运行规划器（可选）：**
```bash
source install/setup.bash
python3 src/dummy_tools/moveit_rviz_planner.py
```

---

## 常见问题

### USB 权限问题
如果遇到 USB 设备无法访问，请检查：
1. udev 规则是否正确配置
2. VID 和 PID 是否匹配
3. 是否重新插拔了设备

### MoveIt 规划失败
1. 检查关节限位是否合理
2. 确认目标位姿是否在工作空间内
3. 查看 RViz 中是否有碰撞

### 编译错误
确保所有依赖包已正确安装：
```bash
rosdep install --from-paths src --ignore-src -r -y
```
