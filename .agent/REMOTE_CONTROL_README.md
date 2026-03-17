# 遥控器底盘控制系统

## 概述

新增的遥控器节点（Remote Control Node）实现了手柄/遥控器对底盘的实时控制。系统支持以下运动方式：

- **前后运动**：向前或向后移动底盘
- **左右运动**：底盘向左或向右平移（适用于全向轮底盘）
- **旋转运动**：底盘原地旋转或移动中旋转

## 系统架构

```
遥控器输入 (Joy Message)
        ↓
┌──────────────────────────────┐
│  remote_control_node         │ ← 新增节点
│  (遥控器转换器)               │
└──────────────────────────────┘
        ↓
    /cmd_vel (Twist)
        ↓
┌──────────────────────────────┐
│  chassis_control_node        │ ← 现有节点
│  (底盘控制器)                 │
└──────────────────────────────┘
        ↓
   /dji_motor_command_advanced
        ↓
┌──────────────────────────────┐
│  motor_control_node          │
│  (电机硬件接口)               │
└──────────────────────────────┘
        ↓
    DJI 电机执行
```

## 节点功能

### RemoteControlNode（遥控器节点）

**主要功能：**
1. 订阅遥控器输入（`/joy`）
2. 将遥控器输入映射为底盘速度命令
3. 支持速度缩放（加速/减速）
4. 支持紧急停止
5. 发布底盘速度命令（`/cmd_vel`）

**输入主题：**
- `/joy` (joy/Joy) - 遥控器/游戏手柄输入

**输出主题：**
- `/cmd_vel` (geometry_msgs/Twist) - 底盘速度命令

## 配置说明

配置文件位置：`config/remote_control_params.yaml`

### 手柄轴映射

针对标准 Xbox 或 Linux 兼容手柄：

```yaml
axis_forward: 1          # 前后运动 - 左摇杆 Y 轴
axis_strafe: 0           # 左右运动 - 左摇杆 X 轴
axis_rotate: 3           # 旋转运动 - 右摇杆 X 轴
```

**标准手柄轴编号：**
| 轴号 | 功能 |
|------|------|
| 0 | 左摇杆 X（左右） |
| 1 | 左摇杆 Y（前后） |
| 2 | 左触发器 |
| 3 | 右摇杆 X（旋转） |
| 4 | 右摇杆 Y |
| 5 | 右触发器 |

### 手柄按钮映射

```yaml
button_speed_up: 5       # RB - 加速
button_speed_down: 4     # LB - 减速
button_stop: 6           # Back - 紧急停止
```

**标准手柄按钮编号：**
| 按钮号 | Xbox | PS4 |
|--------|------|-----|
| 0 | A | Cross |
| 1 | B | Circle |
| 2 | X | Square |
| 3 | Y | Triangle |
| 4 | LB | L1 |
| 5 | RB | R1 |
| 6 | Back | Select |
| 7 | Start | Start |

### 速度参数

```yaml
max_linear_velocity: 2.0      # 最大线速度 m/s
max_angular_velocity: 3.14    # 最大角速度 rad/s
default_speed_scale: 0.5      # 默认速度缩放因子 (50%)
speed_scale_step: 0.1         # 每次按速度按钮调整步长 (10%)
```

### 死区参数

```yaml
deadzone: 0.1                 # 摇杆死区范围，防止漂移
```

## 构建和运行

### 1. 构建项目

```bash
cd /home/toe/test/USB2CAN_motor
colcon build --packages-select motor_control_ros2
```

### 2. 运行遥控器系统

**选项 1：启动完整系统（遥控器 + 底盘控制）**

```bash
# 使用启动脚本
bash src/start_remote_control_system.sh 3

# 或直接运行
source install/setup.bash
ros2 run motor_control_ros2 remote_control_node &
ros2 run motor_control_ros2 chassis_control_node
```

**选项 2：仅启动遥控器节点**

```bash
source install/setup.bash
ros2 run motor_control_ros2 remote_control_node
```

**选项 3：使用自定义配置**

```bash
ros2 run motor_control_ros2 remote_control_node \
  --ros-args \
  --params-file custom_remote_params.yaml
```

### 3. 连接手柄

1. 将遥控器/手柄通过 USB 或蓝牙连接到电脑
2. 验证设备识别：
   ```bash
   ls -l /dev/input/js*
   ```
3. 安装必要的权限（如需要）：
   ```bash
   sudo chmod a+r /dev/input/js*
   ```

## 运动控制说明

### 输入映射

**前后运动（Linear X）：**
- 向前推左摇杆 → 底盘向前移动
- 向后拉左摇杆 → 底盘向后移动

**左右运动（Linear Y）：**
- 左摇杆向左 → 底盘向左平移
- 左摇杆向右 → 底盘向右平移

**旋转运动（Angular Z）：**
- 右摇杆向左 → 底盘逆时针旋转
- 右摇杆向右 → 底盘顺时针旋转

### 速度缩放

- **RB（加速）**：按住 RB 按钮增加速度缩放因子（最多 100%）
- **LB（减速）**：按住 LB 按钮减少速度缩放因子（最少 0%）
- **Back（停止）**：按 Back 按钮立即停止底盘运动

## 调试和监控

### 查看遥控器输入

```bash
ros2 topic echo /joy
```

### 查看速度命令

```bash
ros2 topic echo /cmd_vel
```

### 查看底盘状态

```bash
ros2 topic echo /chassis_state
```

### 记录数据

```bash
ros2 bag record /joy /cmd_vel /chassis_state
```

### 回放数据

```bash
ros2 bag play rosbag2_*
```

## 常见问题

### Q1：手柄无法识别
**A：** 
1. 检查设备是否连接：`ls /dev/input/js*`
2. 检查权限：`sudo chmod a+r /dev/input/js*`
3. 安装 `joystick` 工具进行测试：`jstest /dev/input/js0`

### Q2：摇杆漂移
**A：** 增加死区参数 `deadzone` 的值（建议 0.15-0.2）

### Q3：速度响应太慢或太快
**A：** 调整 `max_linear_velocity` 和 `max_angular_velocity` 参数

### Q4：底盘不按预期运动
**A：** 
1. 检查轴映射是否正确：`ros2 topic echo /joy`
2. 检查电机方向配置是否正确
3. 调整 `chassis_params.yaml` 中的电机方向参数

### Q5：需要自定义速度缩放步长
**A：** 修改 `speed_scale_step` 参数（例如改为 0.05 以获得更精细的控制）

## 扩展功能

### 1. 添加更多按钮功能

编辑 `remote_control_node.cpp`，在 `joyCallback` 中添加：

```cpp
// 例：添加特殊动作按钮
if (msg->buttons[button_special_action]) {
    // 执行特殊动作
}
```

### 2. 支持多个手柄

修改节点以支持多个遥控器输入和优先级管理。

### 3. 与 RViz 可视化集成

使用启动脚本选项 4 启动 RViz，实时显示底盘运动。

## 相关文件

- **源代码：**
  - [remote_control_node.hpp](include/motor_control_ros2/remote_control_node.hpp)
  - [remote_control_node.cpp](src/remote_control_node.cpp)

- **配置文件：**
  - [remote_control_params.yaml](config/remote_control_params.yaml)
  - [chassis_params.yaml](config/chassis_params.yaml)

- **启动脚本：**
  - [start_remote_control_system.sh](../src/start_remote_control_system.sh)

- **底盘控制节点：**
  - [chassis_control_node.cpp](src/chassis_control_node.cpp)

## 性能指标

- **发布频率：** 50 Hz（可配置）
- **摇杆死区：** 0.1（可配置）
- **速度缩放范围：** 0-100%（10% 步长）
- **最大线速度：** 2.0 m/s（可配置）
- **最大角速度：** 3.14 rad/s（可配置）

## 许可证

MIT License

## 工程诊断补充说明

在查看项目日志中的“通信失败率”时，需先确认统计口径：
- `fails_total` 往往表示**单次尝试失败次数**
- 如果后续重试成功，则不应视为“最终控制失败”

建议统一区分：
- 首次失败率
- 重试恢复率
- 最终失败率

否则容易把“链路偏紧、但已恢复”的情况误判为“功能不可用”。

