# 最终方案 Plan

<!-- usage-note -->

## 问题一：串口热插拔修复

### 状态：✅ 已完成

### 修改文件
- `src/motor_control_ros2/src/hardware/serial_interface.cpp`

### 修改内容
在 `sendRecvAccumulate()` 和 `sendRecv()` 的 `send()` 前增加：
```cpp
tcflush(fd_, TCIFLUSH);  // 清空内核 RX 缓冲区
```

### 原理
RS485 半双工请求-响应协议下，发送前不可能有有效的待接收数据。
`tcflush` 清除的只可能是：断电期间的噪声/残留/不完整帧。

### 验证
1. `colcon build --packages-select motor_control_ros2`
2. 启动 motor_control_node
3. 确认 3 台电机在线
4. **不关节点**，断电所有电机
5. 重新上电
6. 观察 3 台电机是否全部恢复在线

---

## 问题二：Delta 机械臂无法到达目标位置

### 状态：⚠️ 需要用户操作（校准 + 调参）

### 根因
1. **kp=0.02 极度偏小** — 产生的力矩（~0.01 Nm）连静摩擦都无法克服
2. **arm_motor_2 offset=0.063 可疑** — 与其他电机（0.639, 0.872）差异过大

### 建议修改：delta_arm.yaml

```yaml
motor:
  kp: 0.50    # 原 0.02，提高 25 倍使电机能到达目标
  kd: 0.10    # 原 0.08，适当提高防止振荡
```

### Offset 重新校准步骤
1. 手动将机械臂放到**已知高度** h（建议用最低位 h_min=0.171m）
2. 启动 motor_control_node，从 monitor 读取 3 个电机的 position (rad)
3. 计算理论运动学角：`θ_rad = inverseKinematics(h) × π / 180`
4. 计算每个电机的 offset：`offset = position × direction − θ_rad`
5. 更新 delta_arm.yaml 的 `motor_offsets_rad`

### 注释修正
```yaml
motor_offsets_rad:
  - 0.639  # arm_motor_1 (36.6°) — 原注释 -15.4° 有误
  - ???    # arm_motor_2 — 需重新校准
  - 0.872  # arm_motor_3 (50.0°) — 原注释 -2.1° 有误
```

### 安全注意事项
- 调参前确保机械臂周围无障碍物
- kp 不要一次性跳到很大值，从 0.5 开始观察
- 如果振荡，先增大 kd 再增大 kp

---

## 会议纪要索引
- [问题一会议纪要](meeting_serial_hotplug.md)
- [问题二会议纪要](meeting_delta_arm_position.md)
- [高度 Bug 原始分析](delta_arm_height_bug_analysis.md)
不
---

## 问题三：单圈绝对编码器（内圈）与机械臂解算（外圈）冲突

### 状态：✅ 已完成（代码与配置均已落地）

### 会议摘要（主Agent + 3个Sub-Agent）
1. **开发子代理（激进方案）**：直接在现有逆解上继续调 `offset`，让高度看起来匹配。
2. **测试子代理（否决）**：否决。原因是上电初值随机落在单圈不同相位，单调性不稳定，单靠 `offset` 无法保证每次一致。
3. **架构子代理（折中方案）**：增加“上电两阶段引导流程”，先走绝对位置定姿，再启用机械臂外圈解算。
4. **最终共识**：三方一致同意采用“**start 位 → ready 位 → 解算控制**”流程，并强制每次上电执行。

### 已实现方案
- 在 `delta_arm.yaml` 增加 `startup_sequence`：
  - `start_positions_rad`: `[0.669, 0.846, 0.886]`
  - `ready_positions_rad`: `[1.261, 1.690, 1.689]`
  - `enabled: true`
- `delta_arm_node` 新增引导状态机：
  - `MOVE_TO_START` -> `MOVE_TO_READY` -> `DONE`
- 引导未完成前，拒绝高度/回零命令（仅允许空闲）。
- 引导完成后，才进入正常逆解与高度控制。

### 修改文件
- `src/motor_control_ros2/include/motor_control_ros2/delta_arm_node.hpp`
- `src/motor_control_ros2/src/delta_arm_node.cpp`
- `src/motor_control_ros2/config/delta_arm.yaml`

### 验证步骤
1. 上电后启动节点，观察日志：
   - 先到 start 位
   - 再到 ready 位
   - 最后提示“进入机械臂解算控制”
2. 引导阶段发送高度命令应被拒绝（日志告警）。
3. 引导完成后发送高度命令，应正常动作并稳定收敛。

---

## 问题四：`motor_offsets_rad` 是否应替换成 `ready_positions_rad`

### 状态：✅ 已定稿（不替换）

### 会议摘要（主Agent + 3个Sub-Agent）
1. **开发子代理（激进）**：建议直接把 `motor_offsets_rad` 改成 ready 三个值，简化配置。
2. **测试子代理（否决）**：否决。ready 位是“启动过程的目标点”，不是“坐标系映射常量”；混用会导致解算漂移。
3. **架构子代理（调和）**：保留“双参数体系”并明确职责边界：
  - `startup_sequence.ready_positions_rad`：一次性引导位
  - `motor_offsets_rad`：长期映射参数（物理角到运动学角）
4. **最终共识**：三方一致同意“不替换”，并建议仅在标定流程后更新 `motor_offsets_rad`。

### 最终方案
- `motor_offsets_rad` 保持“映射参数”语义，不改成 ready 值。
- `ready_positions_rad` 仅用于上电引导状态机，不参与长期映射。
- 若需重标定 offset：
  1. 完成上电引导（到 ready）
  2. 在已知高度/角度姿态采样物理角
  3. 用 `offset = physical_rad * direction - kinematic_rad` 逐电机计算
  4. 仅更新 `motor_offsets_rad`

---

## 问题五：机械臂模块重写为 `delta_arm_manager`

### 状态：✅ 已完成

### 会议摘要（主Agent + 3个Sub-Agent）
1. **开发子代理（激进）**：直接删除旧 Delta 与手柄逻辑，一次性切到新状态机。
2. **测试子代理（否决）**：否决“一步到位”上线。要求保留最小可回滚路径，并先完成消息与配置兼容检查。
3. **架构子代理（折中）**：同意“删除旧逻辑”，但分 4 步实施：
  - 步骤A：先落地新 msg 与新配置
  - 步骤B：实现 `delta_arm_manager` 状态机（Init/SoftLanding/Ready/Execute）
  - 步骤C：移除旧 Delta 节点与手柄到机械臂发布链路
  - 步骤D：编译+联调+发布 ready 信号
4. **最终共识**：三方一致同意按折中方案执行。

### 新节点目标
- 节点名：`delta_arm_manager`
- 控制对象：3 路 GO8010
- 核心状态机：
  - `INIT`
  - `SOFT_LANDING`
  - `READY`
  - `EXECUTE`

### 新配置文件（`arm_config.yaml`）
- `initialization.downward_torque`
- `initialization.landing_timeout`
- `pd.kp`
- `pd.kd`
- `motion_profile.max_velocity`
- `safety.max_height`
- （建议补充）`control_frequency`
- （建议补充）`landing_angle_threshold`（默认 0.02 rad）
- （建议补充）`landing_stable_duration`（默认 0.5 s）

### 新消息
- `ArmTarget.msg`
  - `float64[3] target_angles`
  - `bool execute`

### 关键技术决策
1. **Soft Landing 判定**
  - 修订为“连续 0.5s 三电机角度变化量都小于 0.02rad”作为完成条件。
  - 即判定静止/落稳，不再使用绝对角度阈值。
2. **Ready 通知**
  - 发布 `std_msgs/String` 到 `/delta_arm/ready`，内容：`READY`。
3. **梯形速度规划**
  - 采用离散帧积分：`pos_next = pos_now + clamp(err, ±max_velocity*dt)`。
  - 每帧同步下发 3 路命令。
4. **同步控制频率**
  - 控制定时器固定频率执行（默认 200Hz，可配置）。
5. **解耦坐标设计**
  - 初始化解耦后，三电机控制坐标起点强制置为 `0 rad`（不读取真实绝对角作为控制零点）。
  - 高度变化由相对增量驱动：外圈只计算 `Δrad`。
  - 三个电机每一控制周期的目标增量必须相同：`Δrad_1 = Δrad_2 = Δrad_3`。
  - 三电机目标统一表达为：`target_i = target_i_prev + Δrad`。

### 删除/替换范围（实施时执行）
- 删除旧 Delta 机械臂节点及其旧消息依赖。
- 删除手柄到机械臂的发布逻辑（保留底盘/其它控制链）。
- 新增 `delta_arm_manager` 可执行目标并替换 CMake 注册。

### 口径已确认
- 解耦初始化弧度固定 0。
- 控制只看变化量，不看绝对角。
- 三电机变化量始终一致。

---

## 问题六：`ArmTarget.msg` 导致编译失败

### 状态：✅ 已修复

### 根因
- `float target64_angles` 不是合法 ROS2 msg 类型定义（`float` 非法）。
- 字段也与既定接口不一致，正确应为 `float64[3] target_angles`。

### 修复
- 将 `src/motor_control_ros2/msg/ArmTarget.msg` 修改为：
  - `float64[3] target_angles`
  - `bool execute`

### 验证
1. `colcon build --packages-select motor_control_ros2`
2. `ros2 interface show motor_control_ros2/msg/ArmTarget`
3. 确认字段为：
   - `std_msgs/Header header`
   - `float64[3] target_angles`
   - `bool execute`

---

## 问题七：`motor_control_node.cpp` 语法污染导致编译失败

### 状态：✅ 已修复

### 根因
- 变量声明被污染：`doublemotor_id actual_control_freq_ = 0.0;`
- 类结尾被污染：`};motor_id`

### 修复
- 改为 `double actual_control_freq_ = 0.0;`
- 改为正常类结尾 `};`

### 验证
1. `colcon build --packages-select motor_control_ros2`
2. 确认不再出现 `does not name a type` 与 `actual_control_freq_ was not declared`。

---

## 问题八：`motor_control_node.cpp` 缺少命名空间闭合导致编译失败

### 状态：✅ 已修复

### 根因
-  `namespace motor_control {` 未在文件末尾闭合，触发：
  - `expected '}' at end of input`

### 修复
- 在文件末尾补充：
  - `}  // namespace motor_control`
  - `main()` 入口函数（创建并 spin `MotorControlNode`）

### 验证
1. `colcon build --packages-select motor_control_ros2`
2. 确认不再出现 `expected '}' at end of input`。

---

## 问题九：3 台电机不同步——逐个站起来（2026-03-16 委员会决议）

### 状态：🔧 修复中

### 委员会诊断（3 Sub-Agent 一致通过）

**根因三层叠加：**

| 层级 | 根因 | 影响 |
|------|------|------|
| **层级 1（主因）** | `downward_torque = -0.05 Nm` 转子侧，输出端仅 0.317 Nm ≈ 静摩擦力矩边界 | 摩擦不同的电机下降速度完全不同 |
| **层级 2** | 串口顺序阻塞 30ms，实际频率 ~33Hz | 着陆检测"幻影稳定" bug |
| **层级 3** | `allMotorsLanded()` 用位置差分而非速度反馈 | 缓慢运动时过早误判着陆 |

### 修复措施

#### 🔴 P0（立即执行）
1. **增大力矩**: `downward_torque` -0.05 → -0.15（输出端 0.95 Nm，远超静摩擦上限）
2. **SOFT_LANDING 加阻尼**: 发送命令 kd=0.05（τ = τ_ff + Kd(0-ω)，自动限速）
3. **着陆判定改用速度**: `abs(current_velocity) < 0.3 rad/s` 替代位置差分

#### 🟠 P1（串口优化）
4. `wait_ms` 2→0, `timeout_ms` 8→3（每台电机耗时 10ms→3ms）

#### 🟡 P2（EXECUTE 调优）
5. `kp` 0.40→0.20, `kd` 0.02→0.05（kp/kd=4 接近临界阻尼）

### 修改文件
- `config/arm_config.yaml`
- `src/delta_arm_manager_node.cpp`
- `src/motor_control_node.cpp`

---

## 问题十：`fails_total` 看起来很高，但不等于最终失败率

### 状态：🔍 已澄清口径

### 结论
- `arm_motor_2: 638/1400`
- `arm_motor_3: 441/1400`

这两个数字更可能表示：
- **首次通信失败/触发重试的比例**
而不是：
- **最终命令执行失败的比例**

因为当前现象已明确为“**通过重试恢复 ✅**”。

### 正确理解
- `638/1400 ≈ 45%`：约 45% 的控制周期第一次 `sendRecv` 没成功
- 若第二/第三次重试成功，则该周期**最终并未失败**
- 因此该指标本质更接近“**重试率**”，不是“**最终失败率**”

### 可能原因
1. 串口超时窗口仍偏紧
2. USB-RS485 适配器存在调度/延迟抖动
3. RS485 半双工收发切换时序较边缘
4. 线缆、供电、接地或噪声导致首帧偶发损坏
5. 顺序轮询下，后两路更容易出现首轮超时

### 后续应统计的指标
- `first_try_fail_total`
- `retry_recovered_total`
- `final_fail_total`
- `timeout_total`
- `parse_fail_total`
- `consecutive_fail_max`

### 判断标准
- 若 `final_fail_total` 很低，且电机在线稳定、动作无异常，则当前问题主要是**链路裕量不足**，不是功能性故障。
- 若 `final_fail_total` 也高，才说明需要继续处理物理层或超时参数。

---

## 问题十一：motor_2/3 首次通信失败率 31-45% — 串口超时太紧

### 状态：✅ 已修复

### 根因
- `wait_ms=2, timeout_ms=8` 对 ttyUSB1/2 的 USB-RS485 适配器偏紧
- motor_1 换到 ttyUSB3 后 0% 失败，说明不同适配器响应延迟差异大
- 首次超时后重试能成功，证明不是协议/硬件故障，纯粹是时间窗不够

### 修复
- `wait_ms` 2 → 4（给适配器收发切换更多时间）
- `timeout_ms` 8 → 12（给慢适配器留余量）
- 在 `arm_config.yaml` 新增 `serial.wait_ms` / `serial.timeout_ms` 可配置

### 频率影响
- 单台电机轮询耗时：~10ms → ~16ms
- 3台总计：~30ms → ~48ms
- 等效控制频率：~33Hz → ~20Hz（机械臂场景足够）

### 验证
1. 重新编译运行
2. 观察 `fails_total`，预期 motor_2/3 首次失败率降到 <5%
3. 如仍偏高，可继续放宽到 `wait_ms=5, timeout_ms=15`

---

## 问题十二：构建问题快速排查（固定流程）

### 状态：🔄 执行中

### 固定命令
1. `colcon build --packages-select motor_control_ros2 --event-handlers console_direct+`
2. `colcon build --packages-select motor_control_ros2 --cmake-clean-cache`
3. `rm -rf build/motor_control_ros2 install/motor_control_ros2 log && colcon build --packages-select motor_control_ros2`

### 重点检查
- `msg` 字段类型是否为 ROS2 合法类型（如 `float64`、`float64[3]`）
- `CMakeLists.txt` 是否包含新源文件与 `rosidl` 生成项
- `package.xml` 是否声明对应依赖
- 节点源文件是否闭合 `namespace` 且包含 `main()`

---

## 问题十三：`gear_ratio` 参数语义

### 定义
`gear_ratio` = 电机转子转角 / 关节输出转角（减速比）。

### 换算关系
- `joint_position = motor_position / gear_ratio`
- `joint_velocity = motor_velocity / gear_ratio`
- `joint_torque ≈ motor_torque * gear_ratio * efficiency`

### 配错后现象
- 位置比例错误（指令 10° 实际不是 10°）
- 速度估计偏差
- 力矩/负载判断失真，可能影响控制稳定性

---

## 问题十四：上电后无命令电机就动（残留命令 bug）

### 状态：✅ 已修复

### 根因
- `delta_arm_manager` 启动后立即进入 INIT→SOFT_LANDING，马上发命令
- 但此时电机反馈尚未到达，`zero_positions_` 全为 0
- 导致发出的位置命令与电机实际位置不符，电机瞬间跳动

### 修复
- 新增 `WAIT_FEEDBACK` 状态，作为状态机第一阶段
- 在所有 3 台电机都返回有效反馈前，**不发送任何命令**
- 电机在无命令期间保持 brake/自由状态

### 验证
- 上电后电机应保持静止，直到日志出现"3台电机反馈就绪"

---

## 问题十五：重力补偿前馈接口（用户自行调试）

### 状态：✅ 接口已就绪

### 新增配置
```yaml
initialization:
  gravity_compensation_torque: 0.0  # 用户自行调试
```

### 使用方法
- READY 和 EXECUTE 阶段的 `torque_ff` 会加上此值
- 正值向上抵抗重力
- 建议从 0.05 开始，观察 READY 状态是否下沉，逐步增大
- 调试时注意：这是**转子侧力矩**，输出端 = 值 × gear_ratio（GO8010 约 6.33）

---

## 问题十：传输堵塞 — 串口I/O阻塞主控制循环

### 状态：🔧 进行中（2026-03-17）

### 根因分析
- `controlLoop()`（200Hz=5ms周期）中 `writeUnitreeNativeMotors()` 顺序轮询3个串口电机
- 每个电机通信耗时 ~4-16ms（wait_ms=4 + timeout_ms=12）
- 3电机最少 12ms，最坏含重试 192ms → 远超 5ms 周期
- SingleThreadedExecutor 被 I/O 锁死，ROS2 订阅回调无法处理 → 命令堆积覆盖

### 修复方案
1. **P0**: 为每个串口接口创建独立通信线程，三串口并行收发
2. **P1**: timeout_ms 从 12 降至 8
3. **P2**: publishCommand 添加 torque_ff 限幅保护

### 修改文件
- `src/motor_control_ros2/src/motor_control_node.cpp`
- `src/motor_control_ros2/src/delta_arm_manager_node.cpp`
- `src/motor_control_ros2/config/arm_config.yaml`

### 预期效果
- 控制循环恢复真正 200Hz（<2ms/周期）
- 串口通信频率 ~200Hz（三路并行，每路 ~4ms）
- ROS 回调不再被饿死，命令即时送达电机
