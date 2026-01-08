# launch/virtual_motor.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('virtual_motor_device')
    
    return LaunchDescription([
        # 1. 启动虚拟CAN接口
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'sudo modprobe vcan && '
                'sudo ip link add dev vcan0 type vcan && '
                'sudo ip link set up vcan0 && '
                'echo "虚拟CAN接口 vcan0 已启动"'
            ],
            output='screen'
        ),
        
        # 等待CAN接口就绪
        TimerAction(
            period=2.0,
            actions=[
                # 2. 启动CAN接口桥接
                Node(
                    package='socketcan_interface',
                    executable='socketcan_bridge',
                    name='socketcan_bridge',
                    parameters=[{
                        'interface': 'vcan0',
                        'bitrate': 1000000,
                    }],
                    output='screen'
                ),
                
                # 3. 启动虚拟电机设备
                Node(
                    package='virtual_motor_device',
                    executable='virtual_motor_device_node',
                    name='virtual_motor',
                    parameters=[
                        os.path.join(pkg_path, 'config', 'virtual_motor.yaml')
                    ],
                    output='screen',
                    arguments=['--ros-args', '--log-level', 'info']
                ),
                
                # 4. 启动CANopen主站
                Node(
                    package='canopen_master',
                    executable='canopen_master_node',
                    name='canopen_master',
                    parameters=[{
                        'master_config': os.path.join(pkg_path, 'config', 'master_config.yaml'),
                        'can_interface': 'vcan0',
                        'node_id': 127,
                    }],
                    output='screen'
                ),
                
                # 5. 启动状态监控节点
                Node(
                    package='virtual_motor_device',
                    executable='motor_monitor_node',
                    name='motor_monitor',
                    output='screen'
                ),
                
                # 6. 启动测试控制器（可选）
                TimerAction(
                    period=5.0,
                    actions=[
                        Node(
                            package='virtual_motor_device',
                            executable='test_controller_node',
                            name='test_controller',
                            output='screen'
                        )
                    ]
                )
            ]
        )
    ])