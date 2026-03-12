std_msgs/Header header

# 控制模式
uint8 control_mode  # 0: 任务空间控制, 1: 关节空间控制

# 任务空间目标（control_mode=0时使用）
geometry_msgs/Point target_position     # 末端目标位置 (x, y, z)
float64 target_orientation              # 末端目标姿态（弧度）
geometry_msgs/Vector3 target_velocity   # 末端目标速度 (vx, vy, 0)

# 关节空间目标（control_mode=1时使用）
float64 left_main_motor     # 左臂主电机角度（弧度）
float64 left_sub_motor      # 左臂副电机角度（弧度）
float64 right_main_motor    # 右臂主电机角度（弧度） 
float64 right_sub_motor     # 右臂副电机角度（弧度）

# 控制参数（PID + 前馈力矩）
float64[4] kp          # 刚度系数 [左主, 左副, 右主, 右副]
float64[4] kd          # 阻尼系数
float64[4] torque_ff   # 前馈力矩（Nm）
