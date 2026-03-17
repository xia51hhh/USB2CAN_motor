std_msgs/Header header

# 目标相对增量（3路 GO8010 电机，弧度）
# 解耦模式要求三路值相同，表示统一 Δrad
float64[3] target_angles

# 执行标志：true 时进入 EXECUTE 状态，false 时忽略
bool execute
