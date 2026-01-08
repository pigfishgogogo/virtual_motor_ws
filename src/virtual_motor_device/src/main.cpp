// src/main.cpp
#include "rclcpp/rclcpp.hpp"
#include "virtual_motor_device/virtual_motor_device.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建节点选项
    rclcpp::NodeOptions options;
    
    // 设置参数文件（如果有）
    std::vector<std::string> args;
    if (argc > 1) {
        args.push_back("--ros-args");
        args.push_back("--params-file");
        args.push_back(argv[1]);
    }
    
    options.arguments(args);
    
    // 创建虚拟电机设备节点
    auto node = std::make_shared<VirtualMotorDevice>(options);
    
    RCLCPP_INFO(node->get_logger(), "虚拟电机设备节点启动");
    
    // 运行节点
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}