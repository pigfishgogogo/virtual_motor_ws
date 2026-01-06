#!/usr/bin/env python3
# scripts/test_canopen_motor.py
import subprocess
import time
import struct

def send_can_command(can_id, data):
    """使用cansend发送CAN帧"""
    hex_data = ''.join(f'{byte:02X}' for byte in data)
    cmd = f'cansend vcan0 {can_id:X}#{hex_data}'
    print(f"发送: {cmd}")
    subprocess.run(cmd, shell=True)

def test_nmt_commands():
    """测试NMT命令"""
    print("=== 测试NMT命令 ===")
    
    # 1. 进入预运行状态
    send_can_command(0x000, [0x80, 0x01])  # 进入预运行
    time.sleep(0.5)
    
    # 2. 启动节点
    send_can_command(0x000, [0x01, 0x01])  # 启动节点1
    time.sleep(0.5)

def test_sdo_commands():
    """测试SDO命令"""
    print("\n=== 测试SDO命令 ===")
    
    # 1. 读取设备类型 (0x1000)
    send_can_command(0x601, [0x40, 0x00, 0x10, 0x00])  # 读取0x1000
    time.sleep(0.5)
    
    # 2. 读取状态字 (0x6041)
    send_can_command(0x601, [0x40, 0x41, 0x60, 0x00])
    time.sleep(0.5)
    
    # 3. 写控制字: 准备上电 (0x6040 = 0x0006)
    send_can_command(0x601, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00])
    time.sleep(0.5)
    
    # 4. 写控制字: 上电 (0x6040 = 0x0007)
    send_can_command(0x601, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00])
    time.sleep(0.5)
    
    # 5. 写控制字: 运行使能 (0x6040 = 0x000F)
    send_can_command(0x601, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00])
    time.sleep(0.5)
    
    # 6. 设置目标速度 (0x60FF = 1000 RPM)
    target_speed = int(1000 * 100)  # 转换为0.01 RPM单位
    data = struct.pack('<I', target_speed)
    send_can_command(0x601, [0x23, 0xFF, 0x60, 0x00] + list(data))
    time.sleep(1.0)

def test_pdo_commands():
    """测试PDO命令"""
    print("\n=== 测试PDO命令 ===")
    
    # 发送RPDO1: 控制字 + 目标位置
    control_word = 0x000F  # 运行使能
    target_pos = int(1000 * 1000)  # 1000弧度转换为微弧度
    data = struct.pack('<HI', control_word, target_pos)
    send_can_command(0x201, list(data))
    
    time.sleep(2.0)
    
    # 发送RPDO2: 目标速度
    target_speed = int(500 * 100)  # 500 RPM
    data = struct.pack('<i', target_speed)
    send_can_command(0x301, list(data))
    
    time.sleep(3.0)
    
    # 停止电机
    send_can_command(0x201, [0x06, 0x00, 0x00, 0x00, 0x00, 0x00])

def monitor_can_traffic():
    """监控CAN流量"""
    print("\n=== 监控CAN流量 ===")
    print("在一个新终端中运行: candump vcan0")
    print("或者: ros2 topic echo /virtual_motor/joint_states")

if __name__ == "__main__":
    print("虚拟电机CANopen设备测试脚本")
    print("请确保:")
    print("1. 虚拟CAN接口已启动: sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0")
    print("2. 虚拟电机节点正在运行: ros2 run virtual_motor_device virtual_motor_device_node")
    print("3. 可以打开另一个终端运行: candump vcan0")
    
    input("\n按Enter键开始测试...")
    
    try:
        test_nmt_commands()
        test_sdo_commands()
        test_pdo_commands()
        monitor_can_traffic()
        
        print("\n测试完成！")
        print("可以使用以下命令测试:")
        print("  candump vcan0 - 查看CAN流量")
        print("  ros2 topic echo /virtual_motor/joint_states - 查看电机状态")
        print("  cansend vcan0 601#40006000 - 读取状态字")
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
