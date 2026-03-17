# ROS2 实时控制架构深度分析报告

> **Sub-Agent B** —— ROS2 实时控制架构专家  
> 日期：2026-03-16

---

## 一、架构总览与数据流

```
┌─────────────────────────────────────┐
│       delta_arm_manager (200Hz)     │
│  controlLoop() → publishCommand×3  │
│   每次发 3 条 UnitreeGO8010Command  │
└────────┬────────┬────────┬──────────┘
         │msg_1   │msg_2   │msg_3      ROS2 Topic: unitree_go8010_command (QoS depth=10)
         ▼        ▼        ▼
┌─────────────────────────────────────┐
│    motor_control_node (200Hz)       │
│  unitreeGOCommandCallback:          │
│    → 异步回调写入电机命令缓存       │
│  controlLoop:                       │
│    → writeUnitreeNativeMotors()     │
│      → motor_1: sendRecv 10ms      │
│      → motor_2: sendRecv 10ms      │
│      → motor_3: sendRecv 10ms      │
│      ──────────── 合计 ≥30ms ───── │
└─────────────────────────────────────┘
```

---

## 二、问题 1：ROS2 话题通信延迟分析

### 2.1 同一回调内发布 3 条消息的行为

`delta_arm_manager::controlLoop()` 中：

```cpp
// SOFT_LANDING 阶段
for (size_t i = 0; i < 3; ++i) {
    publishCommand(i, 0.0, 0.0, downward_torque_, 0.0, 0.0);
}
```

每次 `publishCommand()` 内部调用 `cmd_pub_->publish(cmd)`。**关键事实**：

| 特性 | 分析 |
|------|------|
| **发布端行为** | ROS2 `rclcpp` 的 `publish()` 在同一线程连续调用 3 次。在默认 `SingleThreadedExecutor` + `intra-process OFF` 下，消息被序列化后立即推送到 DDS 写入队列，**非阻塞**。 |
| **DDS 层传输** | 3 条消息以 FIFO 顺序进入 DDS writer history。对于同进程/同机通信，使用 shared memory transport 或 loopback UDP，延迟在 **10~200μs** 量级。 |
| **订阅端回调触发** | `motor_control_node` 使用 `SingleThreadedExecutor`。订阅回调 `unitreeGOCommandCallback` 和定时器回调 `controlLoop` **共享同一个执行线程**，回调之间**严格串行**。 |

### 2.2 接收时序分析

```
时间轴 (motor_control_node executor 线程):

─────│──────────────────────│──────────────────────│──────
     │  controlLoop()       │  callback×3          │  controlLoop()
     │  writeUnitreeNative  │  (处理3条消息)         │  ...
     │  阻塞 ≥30ms         │  <1ms                 │
     ├──────────────────────┼──────────────────────┤
     t0                   t0+30ms              t0+31ms
```

**核心发现**：

1. **回调被延迟执行**：当 `controlLoop()` 正在执行 `writeUnitreeNativeMotors()`（阻塞 30ms）时，Executor 线程被占用。即使 3 条消息已经到达 DDS 层，**订阅回调也无法被调度**，直到 `controlLoop()` 返回。

2. **不会乱序**：同一 publisher 在同一线程连续发布，DDS 保证 FIFO 有序传递。回调按 msg_1 → msg_2 → msg_3 顺序执行。

3. **不会在 DDS 层丢失（QoS depth=10）**：发布者和订阅者 QoS history depth 都是 10，队列容量足够。但**逻辑上的"丢失"会发生**——见下一节。

### 2.3 结论

> **ROS2 话题本身不引入显著延迟**（μs 级），真正的问题在于 Executor 线程被 `writeUnitreeNativeMotors()` 长时间阻塞，导致**回调饥饿**。

---

## 三、问题 2：两个 200Hz 定时器的耦合问题

### 3.1 定量分析

| 参数 | 值 |
|------|-----|
| 期望控制周期 | 5ms (200Hz) |
| `writeUnitreeNativeMotors()` 单次耗时 | ≥30ms（3×10ms，失败重试翻倍） |
| 实际可达频率 | **≤33Hz**（最好情况）；有重试时 **≤16Hz** |

### 3.2 motor_control_node 内部的灾难性行为

`motor_control_node` 使用 `SingleThreadedExecutor`（默认），所有定时器和订阅回调**在同一线程串行执行**：

```
┌──────────────────────────────────────────────────────────────────┐
│ 期望: 5ms周期                                                    │
│ 实际:                                                            │
│                                                                  │
│  ┌─controlLoop──────────30ms──────────┐  ┌─cb×n─┐  ┌─control──  │
│  │ writeUnitreeNativeMotors()         │  │回调  │  │Loop()     │
│  │ motor1(10ms) motor2(10ms) motor3   │  │处理  │  │又30ms...  │
│  └────────────────────────────────────┘  └──────┘  └──────────  │
│  0ms                                30ms    31ms      32ms      │
│                                                                  │
│  → 定时器 "漏拍"：200Hz 目标期间，定时器触发了 6 次              │
│  → 但 Executor 忙碌，只能执行 1 次 controlLoop                   │
│  → 其他 5 次被 Executor 丢弃（wall_timer 不补拍）                │
└──────────────────────────────────────────────────────────────────┘
```

### 3.3 命令堆积 vs 丢失

**关键结论：命令并不堆积，而是"覆盖"**——但这反而导致了更隐蔽的问题：

| 场景 | 行为 |
|------|------|
| **命令缓存覆盖** | `unitreeGOCommandCallback` 只是 `setFOCCommand()` 写入缓存。如果在一个 controlLoop 周期内收到多组命令，只有**最后一组生效**。之前的被覆盖。 |
| **反馈过时** | controlLoop 30ms 才跑一次，期间 delta_arm_manager 已经发了 6 组命令。motor_control_node 的反馈（状态发布）也是 30ms 更新一次。delta_arm_manager 基于过时反馈做决策。 |
| **SOFT_LANDING 的特殊性** | 在 SOFT_LANDING 阶段，3 台电机命令内容完全相同（纯力矩 -0.05Nm），所以"覆盖"本身不是问题。**问题在于执行频率太低**。 |

### 3.4 "一个一个站起来" 现象的根本原因

串口通信是**顺序阻塞**的：

```cpp
for (auto& motor : unitree_native_motors_) {
    // motor_1 → sendRecv 10ms → 收到反馈 → 更新状态 → 发布状态
    // motor_2 → sendRecv 10ms → 收到反馈 → 更新状态 → 发布状态
    // motor_3 → sendRecv 10ms → 收到反馈 → 更新状态 → 发布状态
}
```

但所有 3 台电机的状态发布在 `publishStates()` 中**同时进行**（controlLoop 末尾），所以 delta_arm_manager 理论上在同一时刻收到 3 台电机的反馈。

**真正原因是**：`allMotorsLanded()` 判定要求**所有 3 台电机同时满足条件**：

```cpp
bool DeltaArmManager::allMotorsLanded() const {
    for (size_t i = 0; i < 3; ++i) {
        if (!motors_online_[i] || !has_feedback_[i]) return false;
        if (std::abs(current_positions_[i] - last_positions_[i]) >= landing_delta_threshold_) 
            return false;
    }
    return true;
}
```

但由于：
1. **控制频率极低**（~30Hz 而非 200Hz），力矩更新每 30ms 才一次
2. **每个 controlLoop 的 30ms 内，3 台电机的串口通信是顺序的**——motor_1 在 t=0ms 收到命令并回报，motor_3 在 t=30ms 才收到命令并回报
3. **串口通信异常/重试可能导致某台电机间歇性 offline**
4. 一旦某台电机 `has_feedback_[i] == false`（串口超时），`allMotorsLanded()` 立刻返回 false，**稳定计时器被重置**
5. 三台电机争夺串口带宽，通信质量参差不齐

> **结论**：不是电机"一个一个站起来"，而是 `allMotorsLanded()` 的稳定判定**反复被通信失败打断**，导致从 SOFT_LANDING → READY 的转换需要数十秒才能碰巧成功。

---

## 四、问题 3：架构是否应该合并？

### 4.1 当前架构的层次

```
delta_arm_manager                    motor_control_node
  (应用逻辑层)                          (硬件抽象层)
  - 状态机                              - 串口通信
  - 梯形规划                            - 协议编解码
  - PD 控制                             - 多电机管理
       ↕ ROS2 Topic ↕
```

### 4.2 合并 vs 不合并的权衡

| 维度 | 合并 (推荐) | 保持分离 |
|------|-------------|----------|
| **延迟** | ✅ 零通信延迟，命令直接写入电机对象 | ❌ 30ms+ Executor 延迟 |
| **同步性** | ✅ controlLoop 内顺序执行：计算命令→串口收发→更新反馈，一气呵成 | ❌ 两个定时器异步，反馈滞后 |
| **复杂度** | ⚠️ 增加 delta_arm_manager 的依赖 | ✅ 各自独立，可单独测试 |
| **可复用性** | ⚠️ 串口代码耦合到具体应用 | ✅ motor_control_node 可服务多种上层节点 |
| **实际需求** | 当前 **只有 delta_arm_manager 使用 GO8010 电机**，复用性需求不高 | 如果未来有更多应用使用同一批电机，才有意义 |

### 4.3 推荐方案

**短期：不合并节点，但改造通信模式**（见第六节方案）。  
**中期（如果性能仍不够）**：将 Unitree 串口通信提取为独立的 **Hardware Interface** 组件，被 delta_arm_manager 直接以库的形式调用，而不走 ROS2 话题。

> 理由：合并节点是一个大重构，而通过批量消息 + 独立线程改造可以解决 90% 的问题。

---

## 五、问题 4：是否应该合并为 1 条 batch 消息？

### 5.1 当前问题：3 条独立消息

```cpp
// delta_arm_manager: 发 3 条
publishCommand(0, ...);  // msg_1
publishCommand(1, ...);  // msg_2
publishCommand(2, ...);  // msg_3
```

问题：
- 3 次 DDS 序列化 + 传输开销（虽然不大，但无意义）
- motor_control_node 的回调被触发 3 次（每次遍历 `unitree_native_motors_` 做匹配）
- **语义不清晰**：3 条消息应该是一个原子命令组

### 5.2 建议：定义 batch 消息

```
# UnitreeGO8010BatchCommand.msg
std_msgs/Header header
UnitreeGO8010Command[] commands    # 批量命令数组
uint32 sequence_id                 # 可选：序列号，用于跟踪
```

好处：
- **原子性**：一条消息包含 3 台电机的所有命令，不存在"只更新了 2 台"的中间状态
- **减少回调次数**：1 次回调 vs 3 次
- **可扩展**：未来增加更多电机无需改通信逻辑

### 5.3 结论

> ✅ **应该合并为 batch 消息**。但这只是优化的一部分，不是根本解法。根本问题是 controlLoop 阻塞。

---

## 六、完整改进方案

### 方案概述

```
优先级 ①：将串口通信移到独立线程（解除 Executor 阻塞）
优先级 ②：3 电机并行通信 或 batch 写入（降低单次 controlLoop 耗时）
优先级 ③：batch 消息替换 3 条独立消息（降低通信开销）
优先级 ④：频率对齐 + 确定性同步（消除两个定时器的竞争）
```

---

### 改进 ①：串口通信独立线程（核心修复）

**当前问题**：`writeUnitreeNativeMotors()` 在 Executor 线程中阻塞 30ms，导致所有 ROS2 回调被饿死。

**方案**：将串口通信放入独立 `std::thread`，与 Executor 解耦。

```
┌──────────────────────────────────────────────────┐
│ motor_control_node                                │
│                                                   │
│ Executor 线程:                                    │
│   - controlLoop() (200Hz): 计算命令 → 写入共享缓存│
│   - 订阅回调: 更新命令缓存                        │
│   - publishStates(): 发布状态                      │
│                                                   │
│ 串口 IO 线程 (新增):                               │
│   - while(running):                                │
│       读取命令缓存（加锁）                         │
│       顺序/并行 sendRecv 3 台电机                   │
│       写回反馈缓存（加锁）                         │
│       通知 controlLoop 数据就绪                     │
└──────────────────────────────────────────────────┘
```

**关键实现要点**：

```cpp
// 伪代码
class MotorControlNode {
    std::thread serial_io_thread_;
    std::mutex  cmd_mutex_;       // 保护命令缓存
    std::mutex  fb_mutex_;        // 保护反馈缓存
    std::atomic<bool> new_cmd_{false};
    
    void serialIOLoop() {
        while (rclcpp::ok()) {
            // 1. 从共享缓存读取最新命令
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                // 拷贝命令到本地
            }
            
            // 2. 顺序 sendRecv（无需 Executor 线程）
            for (auto& motor : unitree_native_motors_) {
                // sendRecvAccumulate...
            }
            
            // 3. 写回反馈
            {
                std::lock_guard<std::mutex> lock(fb_mutex_);
                // 更新状态
            }
        }
    }
    
    void controlLoop() {
        // 只做轻量操作：读反馈 → 发布状态
        // 耗时 < 1ms，不阻塞 Executor
    }
};
```

> **效果**：controlLoop() 不再阻塞，Executor 可以以真正的 200Hz 处理回调和发布。

---

### 改进 ②：3 台电机并行通信（可选，进一步优化）

**前提**：3 台电机必须在**不同串口设备**上（如果在同一串口总线上则不可并行）。

查看配置：`motor_devices_` 数组。如果 3 台电机在不同 `/dev/ttyUSBx`：

```cpp
// 并行方案
void writeUnitreeNativeMotorsParallel() {
    std::vector<std::future<bool>> futures;
    for (auto& motor : unitree_native_motors_) {
        futures.push_back(std::async(std::launch::async, [&]() {
            // sendRecvAccumulate for this motor
            return ok;
        }));
    }
    for (auto& f : futures) {
        f.get();  // 等待全部完成
    }
}
// 耗时：max(10ms, 10ms, 10ms) = 10ms，而非 30ms
```

如果 3 台电机共享同一串口（半双工总线），**不可并行**，只能顺序。此时应优化单次通信时间：
- 降低 `wait_ms`：4Mbps 波特率下 17 字节发送仅需 ~34μs，电机响应时间约 200μs。`wait_ms=2` (2ms) **过于保守**，可降至 `wait_ms=0` 并使用纯超时等待。
- 降低 `timeout_ms`：从 8ms 降至 3~4ms。
- **预估优化后**：每台 3~5ms，3 台合计 9~15ms，接近可用。

---

### 改进 ③：Batch 消息

新增消息类型：

```
# msg/UnitreeGO8010BatchCommand.msg
std_msgs/Header header
motor_control_ros2/UnitreeGO8010Command[] commands
```

`delta_arm_manager` 端：

```cpp
void publishBatchCommand() {
    auto batch = motor_control_ros2::msg::UnitreeGO8010BatchCommand();
    batch.header.stamp = this->now();
    for (size_t i = 0; i < 3; ++i) {
        auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
        cmd.id = motor_ids_[i];
        cmd.device = motor_devices_[i];
        // ... 设置命令参数
        batch.commands.push_back(cmd);
    }
    batch_cmd_pub_->publish(batch);  // 1 次发布替代 3 次
}
```

`motor_control_node` 端：

```cpp
void unitreeGOBatchCommandCallback(
    const motor_control_ros2::msg::UnitreeGO8010BatchCommand::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (const auto& cmd : msg->commands) {
        // 按 ID + device 路由到对应电机，更新命令缓存
    }
}
```

---

### 改进 ④：频率对齐与确定性同步

**当前问题**：两个节点的 200Hz 定时器没有任何同步，可能出现：
- delta_arm_manager 在 t=0.0ms 发命令
- motor_control_node 在 t=2.5ms 刚跑完上次 controlLoop
- 回调在 t=2.5ms 被处理
- motor_control_node 下次 controlLoop 在 t=5.0ms 才执行
- **总延迟 = 5ms**（最差情况为一个完整控制周期）

**方案 A：事件驱动（推荐）**

motor_control_node 不再用定时器，改为"收到命令就立即执行"：

```cpp
void unitreeGOBatchCommandCallback(const BatchCommand::SharedPtr msg) {
    // 更新命令缓存
    updateCommandCache(msg);
    // 立即触发串口通信（在独立线程中）
    serial_io_cv_.notify_one();
}
```

> 这样 delta_arm_manager 的 200Hz 定时器成为**唯一的节拍源**，消除了两个定时器的相位漂移。

**方案 B：Service 替代 Topic（备选）**

对于要求"确定性应答"的场景，可以用 ROS2 Service：

```
# srv/UnitreeGO8010BatchControl.srv
UnitreeGO8010Command[] commands
---
UnitreeGO8010State[] states
bool success
```

delta_arm_manager 调用 service → motor_control_node 执行串口收发 → 返回反馈。  
**优势**：完全同步，delta_arm_manager 精确知道命令何时被执行。  
**劣势**：service 有额外的 DDS 开销（~200μs），且阻塞 delta_arm_manager 的 controlLoop。

---

## 七、最终推荐的分步实施路线

| 阶段 | 改动 | 预期效果 | 工作量 |
|------|------|---------|--------|
| **Phase 1** | 将 `writeUnitreeNativeMotors()` 移入独立线程 | controlLoop 不再阻塞，回调正常响应 | 中 |
| **Phase 2** | 优化串口时序（降低 wait_ms/timeout_ms） | 单次循环从 30ms 降至 ~12ms | 小 |
| **Phase 3** | 定义 batch 消息，替换 3 条独立消息 | 原子命令 + 减少回调开销 | 小 |
| **Phase 4** | motor_control_node 改为事件驱动 | 消除双定时器相位漂移 | 中 |
| **Phase 5** (可选) | 如 3 电机在不同串口，改为并行通信 | 30ms → 10ms | 中 |

### Phase 1 后的预期架构

```
delta_arm_manager (200Hz)
  │ UnitreeGO8010BatchCommand (1条/周期)
  ▼
motor_control_node
  ├─ Executor 线程: 回调(更新缓存) + controlLoop(发布状态) → <1ms
  └─ Serial IO 线程: sendRecv×3 → 30ms (独立运转，不阻塞 Executor)
```

### 全部完成后的预期架构

```
delta_arm_manager (200Hz, 唯一节拍源)
  │ UnitreeGO8010BatchCommand
  ▼
motor_control_node (事件驱动)
  ├─ Executor 线程: 收到 batch → 更新缓存 → notify IO 线程 → 发布状态
  └─ Serial IO 线程: 被唤醒 → sendRecv×3 (优化时序 ~12ms) → 写回反馈
```

> **预期结果**：
> - 控制延迟从 ~35ms（当前）降至 ~5ms（命令到执行）
> - SOFT_LANDING 阶段不再出现"反复重置稳定计时器"的问题
> - 3 台电机真正同时完成着陆判定
> - 实际控制频率可达真正的 200Hz（对 Executor 而言）

---

## 八、附录：快速验证当前问题的方法

### 验证 1：实际控制频率

```bash
ros2 topic hz /control_frequency
# 或查看日志中的 actual_control_freq_ 值
```

预计看到 ~30Hz 而非 200Hz。

### 验证 2：消息延迟

```bash
ros2 topic delay /unitree_go8010_command
# 对比 header.stamp 与接收时间
```

### 验证 3：SOFT_LANDING 稳定计时器被重置

在 `allMotorsLanded()` 中添加调试日志：

```cpp
if (!motors_online_[i] || !has_feedback_[i]) {
    RCLCPP_WARN_THROTTLE(..., "电机 %zu 不在线或无反馈，稳定计时器将被重置", i);
    return false;
}
```

预计看到频繁的重置日志。
