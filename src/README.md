# 启动虚拟节点
ros2 run virtual_motor_device virtual_motor_device_node 

ros2 run virtual_motor_device virtual_motor_device_node --ros-args -p node_ids:="[1,2,3,4]"

# 运行脚本设置 电机初始状态等 （
python /home/alpha/Desktop/virtual_motor_ws/src/virtual_motor_device/scripts/test_canopen_motor.py


cansend vcan0 601#2B4060000F00
cansend vcan0 601#23FF6000A0860100