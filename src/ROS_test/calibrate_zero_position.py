#!/usr/bin/env python3
"""
宇树电机零点校准工具
将当前位置设置为0度基准点
"""

import rclpy
from rclpy.node import Node
from motor_control_ros2.msg import UnitreeGO8010State
import yaml
import time
import math
from collections import defaultdict

class ZeroCalibrator(Node):
    def __init__(self):
        super().__init__('zero_calibrator')
        
        # 存储每个电机的最新位置
        self.motor_positions = {}
        self.motor_names = ['strike_motor_L1', 'strike_motor_L2', 
                           'strike_motor_R1', 'strike_motor_R2']
        
        # 订阅电机状态
        self.subscription = self.create_subscription(
            UnitreeGO8010State,
            '/unitree_go8010_states',
            self.state_callback,
            10)
        
        print("\n" + "="*60)
        print("  宇树电机零点校准工具")
        print("="*60)
        print("\n正在读取电机当前位置...")
        print("请保持电机在期望的零点位置...")
        print()
        
    def state_callback(self, msg):
        """接收电机状态"""
        if msg.joint_name in self.motor_names:
            self.motor_positions[msg.joint_name] = msg.position
            
    def get_positions(self, timeout=3.0):
        """等待收集所有电机位置"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if len(self.motor_positions) >= len(self.motor_names):
                return True
        
        return False
    
    def update_config(self, reverse_motors=None):
        """更新 motors.yaml 配置文件
        
        Args:
            reverse_motors: 需要反转方向的电机名称列表
        """
        if reverse_motors is None:
            reverse_motors = []
            
        config_path = '/home/rosemaryrabbit/USB2CAN_motor/src/motor_control_ros2/config/motors.yaml'
        
        # 读取现有配置
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # 更新每个电机的 offset 和 direction
        motors_updated = []
        for interface in config.get('serial_interfaces', []):
            if 'motors' in interface:
                for motor in interface['motors']:
                    motor_name = motor['name']
                    if motor_name in self.motor_positions:
                        old_offset = motor.get('offset', 0.0)
                        old_direction = motor.get('direction', 1)
                        current_pos = self.motor_positions[motor_name]
                        
                        # 检查是否需要反转方向
                        if motor_name in reverse_motors:
                            new_direction = -old_direction
                            motor['direction'] = new_direction
                            print(f"  {motor_name}:")
                            print(f"    ⚠️  方向反转: {old_direction} → {new_direction}")
                        else:
                            new_direction = old_direction
                        
                        # 新的offset = 旧offset + 当前位置
                        new_offset = old_offset + current_pos
                        motor['offset'] = round(new_offset, 6)
                        motors_updated.append(motor_name)
                        
                        if motor_name not in reverse_motors:
                            print(f"  {motor_name}:")
                        print(f"    当前读数: {math.degrees(current_pos):8.2f}°  ({current_pos:.4f} rad)")
                        print(f"    旧偏移:   {math.degrees(old_offset):8.2f}°  ({old_offset:.4f} rad)")
                        print(f"    新偏移:   {math.degrees(new_offset):8.2f}°  ({new_offset:.4f} rad)")
                        print()
        
        # 备份原配置
        import shutil
        backup_path = config_path + '.backup'
        shutil.copy2(config_path, backup_path)
        print(f"✓ 已备份原配置到: {backup_path}")
        
        # 写入新配置
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(config, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        
        print(f"✓ 已更新配置文件: {config_path}")
        print(f"✓ 更新了 {len(motors_updated)} 个电机的零点偏移")
        
        return motors_updated


def main():
    rclpy.init()
    calibrator = ZeroCalibrator()
    
    try:
        # 收集电机位置
        success = calibrator.get_positions(timeout=5.0)
        
        if not success:
            print("\n❌ 错误: 未能读取到所有电机位置")
            print(f"   已读取: {list(calibrator.motor_positions.keys())}")
            print(f"   需要的: {calibrator.motor_names}")
            print("\n请确认:")
            print("  1. motor_control_node 正在运行")
            print("  2. 电机已连接并上电")
            print("  3. 话题 /unitree_go8010_states 正在发布数据")
            return
        
        print(f"\n✓ 成功读取所有 {len(calibrator.motor_positions)} 个电机位置\n")
        print("当前位置:")
        print("-" * 60)
        for name, pos in calibrator.motor_positions.items():
            print(f"  {name:20s}: {math.degrees(pos):8.2f}° ({pos:7.4f} rad)")
        print("-" * 60)
        
        # 确认
        response = input("\n是否将当前位置设置为零点? (Y/n): ")
        if response.lower() in ['n', 'no']:
            print("已取消校准")
            return
        
        print("\n正在更新配置...")
        print("="*60)
        # 反转 L1 和 R2 的方向（加负号）
        reverse_motors = ['strike_motor_L1', 'strike_motor_R2']
        print(f"⚠️  将反转以下电机的方向（加负号）: {', '.join(reverse_motors)}\n")
        motors_updated = calibrator.update_config(reverse_motors=reverse_motors)
        print("="*60)
        
        print("\n✅ 零点校准完成！")
        print("\n下一步:")
        print("  1. 重启电机控制节点以应用新配置")
        print("     Ctrl+C 停止当前节点，然后运行:")
        print("     ros2 run motor_control_ros2 motor_control_node")
        print("\n  2. 验证零点:")
        print("     ros2 topic echo /unitree_go8010_states")
        print("     当前位置应该接近 0°")
        print()
        
    except KeyboardInterrupt:
        print("\n已取消校准")
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
