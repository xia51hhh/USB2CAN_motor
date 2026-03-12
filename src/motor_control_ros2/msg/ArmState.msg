std_msgs/Header header

# 末端位姿（击球板）
geometry_msgs/Point plate_position      # 击球板位置 (x, y, z)
float64 plate_orientation               # 击球板姿态（弧度）
geometry_msgs/Vector3 plate_velocity    # 击球板速度 (vx, vy, vz)

# 关节角度
float64 left_upper_arm_angle            # 左臂大臂角度（弧度）
float64 left_elbow_angle                # 左臂肘关节角度（弧度）
float64 right_upper_arm_angle           # 右臂大臂角度（弧度）
float64 right_elbow_angle               # 右臂肘关节角度（弧度）

# 电机状态 [左主, 左副, 右主, 右副]
float64[4] motor_positions    # 电机角度（弧度）
float64[4] motor_velocities   # 电机速度（rad/s）
float64[4] motor_torques      # 电机力矩（Nm）
int8[4] motor_temperatures    # 电机温度（°C）
bool[4] motors_online         # 电机在线状态

# 控制状态
uint8 control_mode            # 当前控制模式
float64 coordination_error    # 双臂协调误差（m）
float64 feasibility_index     # 可行性指数（0-1）
bool in_workspace             # 是否在工作空间内
bool ready_to_strike          # 是否就绪可击球
