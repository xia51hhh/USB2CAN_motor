# USB2CAN 电机控制系统

## 🚀 快速开始

### 方案A：在 Conda 环境中使用 ROS（推荐）

#### 1️⃣ 首次配置
```bash
cd /home/rick/desktop/ros/usb2can/src

# 激活你的 conda 环境
conda activate base  # 或你的其他环境

# 运行配置脚本
bash setup_conda_ros.sh

# 重新激活环境以加载 ROS
conda deactivate
conda activate base
```

#### 2️⃣ 验证配置
```bash
python test_ros_import.py
bash check_ros.sh  # 检查 ROS 环境
```

#### 3️⃣ 运行程序

**方式1: 一键启动（推荐）**
```bash
bash start_ros_system.sh
```
脚本会自动启动 roscore 并提供选项启动控制器/发布器。

**方式2: 手动启动**

```bash
# 终端1: 启动 ROS Master（必须先启动）
roscore

# 终端2: 控制程序
python main.py --ros

# 终端3: 测试发布器（可选）
python ros_target_publisher.py

# 或手动发布
rostopic pub /motor_targets std_msgs/Float32MultiArray "data: [90.0, 180.0]"
```

**非ROS模式：**
```bash
python main.py
```

---

## ⚠️ 常见错误及解决

### 错误1: "Unable to register with master node"
```
Unable to register with master node [http://localhost:11311]: master may not be running yet.
```

**原因:** 没有启动 roscore

**解决:**
```bash
# 方法1: 一键启动
bash start_ros_system.sh

# 方法2: 手动启动
# 终端1
roscore
# 终端2
python main.py --ros
```

### 错误2: "No module named 'rospkg'"
**解决:**
```bash
conda activate base
bash setup_conda_ros.sh
conda deactivate && conda activate base
python test_ros_import.py  # 验证
```

### 错误3: 串口权限不够
**解决:** 代码会自动执行 `sudo chmod 777 /dev/ttyACM0`

---

## 📦 依赖说明

### Conda 环境需要安装：
```bash
pip install rospkg catkin_pkg empy defusedxml netifaces pyserial
```

### 系统需要安装：
```bash
sudo apt install ros-noetic-desktop-full
```

---

## 🎯 核心功能

### 1. 多电机控制
- 支持 1-4 个 GM6020 电机同时控制
- 双环 PID 控制（角度环 + 速度环）
- 可配置每个电机的 PID 参数

### 2. ROS 集成
- 订阅 `/motor_targets` 话题动态更新目标角度
- 完全解耦设计，可独立运行
- 支持 `--ros` 参数切换模式

### 3. 调试工具
```bash
# 检查 ROS 环境
bash check_ros.sh

# 查看原始CAN数据
python debug_can.py

# 测试ROS模块
python test_ros_import.py
```

---

## 📂 文件说明

| 文件 | 功能 |
|------|------|
| `main.py` | 主控制程序 |
| `ros_angle_updater.py` | ROS 角度更新模块 |
| `ros_target_publisher.py` | ROS 测试发布器 |
| `start_ros_system.sh` | 一键启动脚本（自动启动roscore） |
| `check_ros.sh` | ROS 环境诊断脚本 |
| `setup_conda_ros.sh` | Conda 环境 ROS 配置脚本 |
| `test_ros_import.py` | ROS 导入测试 |
| `motor_controller.py` | 单电机控制器 |
| `multi_motor_manager.py` | 多电机管理器 |
| `can_driver.py` | CAN 总线驱动 |
| `dji_motor.py` | DJI 电机协议 |
| `pid.py` | PID 控制器 |

---

## 🔧 ROS 使用详解

### 启动顺序
1. **roscore** (ROS Master) - 必须先启动
2. **控制节点** (main.py --ros) - 订阅话题，控制电机
3. **发布节点** (ros_target_publisher.py) - 发布目标角度

### 话题说明
- **话题名称:** `/motor_targets`
- **消息类型:** `std_msgs/Float32MultiArray`
- **数据格式:** `data: [motor1_angle, motor2_angle, ...]`
- **单位:** 度 (0-360)

### 常用命令
```bash
# 查看所有话题
rostopic list

# 查看话题数据
rostopic echo /motor_targets

# 查看话题信息
rostopic info /motor_targets

# 单次发布
rostopic pub -1 /motor_targets std_msgs/Float32MultiArray "data: [45.0, 135.0]"

# 持续发布 (10Hz)
rostopic pub /motor_targets std_msgs/Float32MultiArray "data: [90.0, 180.0]" -r 10
```

---

## ❓ 常见问题

### Q1: 如何验证 ROS 是否配置成功
**A:** 
```bash
bash check_ros.sh
python test_ros_import.py
```

### Q2: 想在不同 conda 环境使用 ROS
**A:** 每个环境都需要运行一次 `setup_conda_ros.sh`

### Q3: 如何停止所有 ROS 节点
**A:**
```bash
# 停止所有节点
rosnode kill -a

# 停止 roscore
killall -9 roscore rosmaster
```

### Q4: 测试是否能接收到话题
**A:**
```bash
# 终端1: 启动控制器
python main.py --ros

# 终端2: 发布测试数据
rostopic pub /motor_targets std_msgs/Float32MultiArray "data: [180.0, 90.0]" -1

# 观察终端1是否显示 "[ROS更新] 电机1 -> 180.0°"
```

---

## 🔧 高级配置

### 修改 PID 参数
编辑 `main.py` 中的 `MOTOR_CONFIGS`：
```python
MOTOR_CONFIGS = [
    {
        'id': 1,
        'target_angle': 90.0,
        'speed_pid': {'kp': 30.0, 'ki': 1.0, 'kd': 0.0, ...},
        'angle_pid': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, ...}
    },
]
```

### 添加更多电机
在 `MOTOR_CONFIGS` 列表中添加新配置即可。

---

## 📊 性能参数

- 控制频率: 200Hz
- 角度分辨率: 0.044° (8192 counts/360°)
- 速度范围: ±320 RPM
- 电压输出: ±30000

---

## 📞 支持

如有问题，请检查：
1. ✅ roscore 是否运行: `pgrep roscore`
2. ✅ ROS 环境是否正确加载: `bash check_ros.sh`
3. ✅ 串口是否有权限: `ls -l /dev/ttyACM0`
4. ✅ 电机是否正确连接
