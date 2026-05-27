// src/virtual_motor_device.cpp
#include "virtual_motor_device/virtual_motor_device.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;
using namespace canopen;

VirtualMotorDevice::VirtualMotorDevice(const rclcpp::NodeOptions & options)
: rclcpp::Node("virtual_motor_device", options),
  can_socket_(-1),
  can_running_(false)
{
    RCLCPP_INFO(this->get_logger(), "虚拟电机CANopen设备启动（多节点模式）");
    
    // 从参数服务器读取参数
    // this->declare_parameter<std::vector<uint8_t>>("node_ids", std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04});
    this->declare_parameter<std::string>("can_interface", "vcan0");
    this->declare_parameter<double>("max_speed", 3000.0);
    this->declare_parameter<double>("max_torque", 10.0);
    
    // std::vector<int64_t> node_ids_param = this->get_parameter("node_ids").as_integer_array();  // int64_t
    // if (node_ids_param.empty()) {
    //     node_ids_param = {0x01};
    // }
    
    // // 转换为uint8_t
    // for (auto id : node_ids_param) {
    //     if (id >= 1 && id <= 127) {
    //         node_ids_.push_back(static_cast<uint8_t>(id));
    //     } else {
    //         RCLCPP_WARN(this->get_logger(), "节点ID %ld 无效，跳过", id);
    //     }
    // }
    
    // if (node_ids_.empty()) {
    //     RCLCPP_FATAL(this->get_logger(), "没有有效的节点ID");
    //     return;
    // }
    node_ids_ = {1,2,3,4};
    std::string can_interface = this->get_parameter("can_interface").as_string();
    max_speed_ = this->get_parameter("max_speed").as_double();
    max_torque_ = this->get_parameter("max_torque").as_double();
    
    RCLCPP_INFO(this->get_logger(), "节点IDs: ");
    for (auto id : node_ids_) {
        RCLCPP_INFO(this->get_logger(), "  节点ID: 0x%02X", id);
    }
    RCLCPP_INFO(this->get_logger(), "CAN接口: %s", can_interface.c_str());
    
    // 初始化CAN接口
    if (!initCANInterface(can_interface)) {
        RCLCPP_FATAL(this->get_logger(), "CAN接口初始化失败");
        return;
    }
    
    // 初始化所有虚拟电机
    initVirtualMotors();
    
    // 创建ROS2发布器 - 每个节点一个
    for (auto node_id : node_ids_) {
        std::string topic_name = "/virtual_motor/node_" + std::to_string(node_id) + "/joint_states";
        auto pub = this->create_publisher<sensor_msgs::msg::JointState>(topic_name, 10);
        joint_state_pubs_[node_id] = pub;
        
        // 创建服务 - 每个节点一个
        std::string service_name = "/virtual_motor/node_" + std::to_string(node_id) + "/enable";
        auto srv = this->create_service<std_srvs::srv::SetBool>(
            service_name,
            [this, node_id](const std_srvs::srv::SetBool::Request::SharedPtr request,
                           std_srvs::srv::SetBool::Response::SharedPtr response) {
                handleEnableService(node_id, request, response);
            });
        enable_services_[node_id] = srv;
    }
    
    // 启动CAN读写线程
    can_running_ = true;
    can_read_thread_ = std::thread(&VirtualMotorDevice::canReadThread, this);
    can_write_thread_ = std::thread(&VirtualMotorDevice::canWriteThread, this);
    
    // 创建定时器
    sim_timer_ = this->create_wall_timer(
        10ms, std::bind(&VirtualMotorDevice::simulateAllMotors, this));
    
    publish_timer_ = this->create_wall_timer(
        20ms, std::bind(&VirtualMotorDevice::publishAllJointStates, this));
    
    // 发送启动报文（心跳）给所有节点
    for (auto node_id : node_ids_) {
        sendCANFrame(canopen::NMT_ID, {0x01, node_id});  // NMT启动命令
        RCLCPP_INFO(this->get_logger(), "发送NMT启动命令到节点 0x%02X", node_id);
    }
    
    RCLCPP_INFO(this->get_logger(), "虚拟电机CANopen设备初始化完成（共 %zu 个节点）", node_ids_.size());
}

VirtualMotorDevice::~VirtualMotorDevice() {
    can_running_ = false;
    
    if (can_read_thread_.joinable()) {
        can_read_thread_.join();
    }
    
    if (can_write_thread_.joinable()) {
        can_write_thread_.join();
    }
    
    if (can_socket_ >= 0) {
        close(can_socket_);
    }
    
    RCLCPP_INFO(this->get_logger(), "虚拟电机CANopen设备关闭");
}

bool VirtualMotorDevice::initCANInterface(const std::string& interface) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    
    // 创建CAN socket
    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "创建CAN socket失败: %s", strerror(errno));
        return false;
    }
    
    // 设置非阻塞
    int flags = fcntl(can_socket_, F_GETFL, 0);
    fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);
    
    // 获取接口索引
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "获取接口索引失败: %s", strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    // 绑定socket
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "绑定CAN socket失败: %s", strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "CAN接口 %s 初始化成功", interface.c_str());
    return true;
}

void VirtualMotorDevice::canReadThread() {
    struct can_frame frame;
    struct pollfd fds[1];
    
    fds[0].fd = can_socket_;
    fds[0].events = POLLIN;
    
    while (can_running_) {
        int ret = poll(fds, 1, 100);
        
        if (ret > 0 && (fds[0].revents & POLLIN)) {
            if (receiveCANFrame(frame)) {
                processCANFrame(frame);
            }
        }
    }
}

void VirtualMotorDevice::canWriteThread() {
    while (can_running_) {
        std::this_thread::sleep_for(10ms);
        
        std::lock_guard<std::mutex> lock(can_mutex_);
        if (!can_write_queue_.empty()) {
            auto frame = can_write_queue_.front();
            can_write_queue_.erase(can_write_queue_.begin());
            
            struct can_frame can_frame;
            can_frame.can_id = frame.first;
            can_frame.can_dlc = std::min<__u8>(frame.second.size(), 8);
            memcpy(can_frame.data, frame.second.data(), can_frame.can_dlc);
            
            if (write(can_socket_, &can_frame, sizeof(can_frame)) != sizeof(can_frame)) {
                RCLCPP_WARN(this->get_logger(), "发送CAN帧失败");
            }
        }
    }
}

void VirtualMotorDevice::sendCANFrame(uint32_t can_id, const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can_write_queue_.push_back({can_id, data});
}

bool VirtualMotorDevice::receiveCANFrame(struct can_frame& frame) {
    int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    
    if (nbytes == sizeof(struct can_frame)) {
        return true;
    }
    
    return false;
}

void VirtualMotorDevice::processCANFrame(const struct can_frame& frame) {
    uint32_t can_id = frame.can_id & CAN_EFF_MASK;
    
    // 打印接收到的CAN帧（调试用）
    std::stringstream ss;
    ss << "收到CAN帧: ID=0x" << std::hex << std::setw(3) << std::setfill('0') << can_id
       << " DLC=" << std::dec << static_cast<int>(frame.can_dlc) << " Data=";
    for (int i = 0; i < frame.can_dlc; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') 
           << static_cast<int>(frame.data[i]) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    
    // 根据CAN ID分发处理
    if (can_id == canopen::NMT_ID) {
        processNMTFrame(frame);
    } else if ((can_id & 0x780) == 0x600) {
        // SDO请求（主站 -> 从站）
        // 0x600 + node_id，例如 0x601 表示节点1
        uint8_t node_id = can_id & 0x7F;
        RCLCPP_INFO(this->get_logger(), "SDO请求到节点 0x%02X (CAN ID: 0x%03X)", node_id, can_id);
        
        if (std::find(node_ids_.begin(), node_ids_.end(), node_id) != node_ids_.end()) {
            processSDOFrame(frame, node_id);
        } else {
            RCLCPP_WARN(this->get_logger(), "未知节点ID: 0x%02X", node_id);
        }
    } else if ((can_id & 0x780) == 0x580) {
        // SDO响应（从站 -> 主站） - 主站发的，我们不处理
        RCLCPP_INFO(this->get_logger(), "忽略SDO响应帧: 0x%03X", can_id);
    } else if ((can_id & 0x780) == 0x200) {
        // RPDO1（主站 -> 从站）
        uint8_t node_id = can_id & 0x7F;
        if (std::find(node_ids_.begin(), node_ids_.end(), node_id) != node_ids_.end()) {
            processRPDO1(frame, node_id);
        }
    } else if ((can_id & 0x780) == 0x300) {
        // RPDO2
        uint8_t node_id = can_id & 0x7F;
        if (std::find(node_ids_.begin(), node_ids_.end(), node_id) != node_ids_.end()) {
            processRPDO2(frame, node_id);
        }
    } else if ((can_id & 0x780) == 0x700) {
        // 心跳 - 这是从站发的，我们不处理接收
        RCLCPP_INFO(this->get_logger(), "收到心跳帧: 0x%03X", can_id);
    } else {
        RCLCPP_WARN(this->get_logger(), "未处理的CAN帧: 0x%03X", can_id);
    }
}

void VirtualMotorDevice::processNMTFrame(const struct can_frame& frame) {
    if (frame.can_dlc >= 2) {
        uint8_t command = frame.data[0];
        uint8_t target_node_id = frame.data[1];
        
        if (target_node_id == 0) {
            // 广播命令，影响所有节点
            for (auto node_id : node_ids_) {
                handleNMTCommand(command, node_id);
            }
        } else {
            // 单播命令
            if (std::find(node_ids_.begin(), node_ids_.end(), target_node_id) != node_ids_.end()) {
                handleNMTCommand(command, target_node_id);
            }
        }
    }
}

void VirtualMotorDevice::handleNMTCommand(uint8_t command, uint8_t node_id) {
    auto& motor = motors_[node_id];
    
    switch (command) {
        case canopen::NMT_START:
            motor.state = STATE_OPERATIONAL;
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 收到NMT启动命令，进入运行状态", node_id);
            break;
        case canopen::NMT_STOP:
            motor.state = STATE_STOPPED;
            motor.velocity = 0.0;
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 收到NMT停止命令，电机停止", node_id);
            break;
        case canopen::NMT_ENTER_PREOP:
            motor.state = STATE_PRE_OP;
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 进入预运行状态", node_id);
            break;
        case canopen::NMT_RESET_NODE:
            // 复位节点
            motor = VirtualMotorState();
            motor.state = STATE_PRE_OP;
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 收到复位命令", node_id);
            break;
        case canopen::NMT_RESET_COMM:
            // 复位通信
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 收到通信复位命令", node_id);
            break;
    }
    
    // 发送心跳响应
    uint8_t heartbeat = static_cast<uint8_t>(motor.state);
    sendCANFrame(0x700 + node_id, {heartbeat});
}

void VirtualMotorDevice::processSDOFrame(const struct can_frame& frame, uint8_t node_id) {
    if (frame.can_dlc >= 4) {
        uint8_t command = frame.data[0];
        uint16_t index = (frame.data[2] << 8) | frame.data[1];
        uint8_t subindex = frame.data[3];
        
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 收到SDO命令: 0x%02X, 索引: 0x%04X, 子索引: 0x%02X",
                   node_id, command, index, subindex);
        
        if ((command & 0xE0) == canopen::SDO_UPLOAD_INITIATE) {
            // SDO读请求
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: SDO读请求", node_id);
            handleSDORead(index, subindex, node_id);
        } else if ((command & 0xE0) == canopen::SDO_DOWNLOAD_INITIATE) {
            // SDO写请求
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: SDO写请求", node_id);
            
            std::vector<uint8_t> data;
            uint8_t data_size = 4 - ((command >> 2) & 0x03);
            RCLCPP_INFO(this->get_logger(), "数据大小: %d", data_size);
            
            for (int i = 0; i < data_size && (i + 4) < frame.can_dlc; i++) {
                data.push_back(frame.data[4 + i]);
            }
            
            handleSDOWrite(index, subindex, data, node_id);
        } else {
            RCLCPP_WARN(this->get_logger(), "节点 0x%02X: 未知SDO命令: 0x%02X", node_id, command);
        }
    }
}

void VirtualMotorDevice::handleSDORead(uint16_t index, uint8_t subindex, uint8_t node_id) {
    RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 处理SDO读请求 索引:0x%04X 子索引:0x%02X", 
                node_id, index, subindex);
    
    std::vector<uint8_t> data;
    
    if (readFromObjectDictionary(index, subindex, data, node_id)) {
        sendSDOResponse(index, subindex, data, node_id, true);
    } else {
        // 发送错误响应
        std::vector<uint8_t> error_response = {
            canopen::SDO_ABORT,
            static_cast<uint8_t>(index & 0xFF),
            static_cast<uint8_t>(index >> 8),
            subindex,
            0x06, 0x00, 0x00, 0x00  // 错误码: 对象不存在
        };
        sendCANFrame(canopen::SDO_TX_BASE_ID + node_id, error_response);
    }
}

void VirtualMotorDevice::handleSDOWrite(uint16_t index, uint8_t subindex, 
                                       const std::vector<uint8_t>& data, uint8_t node_id) {
    RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 处理SDO写请求, 数据大小: %zu", node_id, data.size());
    
    if (writeToObjectDictionary(index, subindex, data, node_id)) {
        // 发送成功响应
        uint8_t cmd = static_cast<uint8_t>(canopen::SDO_DOWNLOAD_INITIATE | ((4 - data.size()) << 2));
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: SDO写成功, 响应命令: 0x%02X", node_id, cmd);
        
        std::vector<uint8_t> response = {
            cmd,
            static_cast<uint8_t>(index & 0xFF),
            static_cast<uint8_t>(index >> 8),
            subindex
        };
        
        // 复制数据（如果有）
        if (!data.empty()) {
            response.insert(response.end(), data.begin(), data.end());
        }
        
        // 发送到正确的CAN ID
        uint32_t response_id = canopen::SDO_TX_BASE_ID + node_id;
        sendCANFrame(response_id, response);
        
        // 如果是控制字，处理它
        if (index == 0x6040 && subindex == 0) {
            if (data.size() >= 2) {
                uint16_t control_word = (data[1] << 8) | data[0];
                RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 控制字=0x%04X", node_id, control_word);
                processControlWord(control_word, node_id);
            }
        }
    } else {
        // 发送错误响应
        RCLCPP_ERROR(this->get_logger(), "节点 0x%02X: SDO写失败", node_id);
        std::vector<uint8_t> error_response = {
            canopen::SDO_ABORT,
            static_cast<uint8_t>(index & 0xFF),
            static_cast<uint8_t>(index >> 8),
            subindex,
            0x06, 0x00, 0x00, 0x00  // 错误码
        };
        sendCANFrame(canopen::SDO_TX_BASE_ID + node_id, error_response);
    }
}

void VirtualMotorDevice::sendSDOResponse(uint16_t index, uint8_t subindex, 
                                       const std::vector<uint8_t>& data, uint8_t node_id, bool success) {
    std::vector<uint8_t> response;
    
    if (success) {
        uint8_t cmd = static_cast<uint8_t>(canopen::SDO_UPLOAD_INITIATE | ((4 - data.size()) << 2));
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 发送SDO成功响应, 命令: 0x%02X, 数据大小: %zu", 
                   node_id, cmd, data.size());
        
        response.push_back(cmd);
        response.push_back(static_cast<uint8_t>(index & 0xFF));
        response.push_back(static_cast<uint8_t>(index >> 8));
        response.push_back(subindex);
        response.insert(response.end(), data.begin(), data.end());
        
        // 打印数据内容
        std::stringstream ss;
        ss << "响应数据: ";
        for (auto byte : data) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "节点 0x%02X: 发送SDO错误响应", node_id);
        response = {
            canopen::SDO_ABORT,
            static_cast<uint8_t>(index & 0xFF),
            static_cast<uint8_t>(index >> 8),
            subindex,
            0x06, 0x00, 0x00, 0x00  // 对象不存在错误
        };
    }
    
    // 发送到正确的CAN ID: 0x580 + node_id
    uint32_t response_id = canopen::SDO_TX_BASE_ID + node_id;
    RCLCPP_INFO(this->get_logger(), "发送SDO响应到 CAN ID: 0x%03X", response_id);
    sendCANFrame(response_id, response);
}

bool VirtualMotorDevice::readFromObjectDictionary(uint16_t index, uint8_t subindex, 
                                                 std::vector<uint8_t>& data, uint8_t node_id) {
    auto& motor = motors_[node_id];
    uint32_t key = (index << 8) | subindex;
    
    // 检查节点特定的对象字典
    if (motor.object_dictionary.find(key) != motor.object_dictionary.end()) {
        data = motor.object_dictionary[key];
        return true;
    }
    
    // 动态生成一些标准对象
    switch (index) {
        case 0x1000:  // 设备类型
            data = {0x92, 0x01, 0x02, 0x00};  // DS402伺服驱动器
            break;
        case 0x1008:  // 设备名称
        {
            std::string name = "VirtualMotor_" + std::to_string(node_id);
            data.assign(name.begin(), name.end());
            break;
        }
        case 0x1018:  // 厂商信息
            if (subindex == 0) {
                data = {0x02};  // 2个条目
            } else if (subindex == 1) {
                data = {0x01, 0x00, 0x00, 0x00};  // 厂商ID
            } else if (subindex == 2) {
                data = {'R', 'O', 'S', '2', ' ', 'V', 'i', 'r', 't', 'u', 'a', 'l'};
            }
            break;
        case 0x6041:  // 状态字
        {
            uint16_t status = getStatusWord(node_id);
            data = {static_cast<uint8_t>(status & 0xFF), static_cast<uint8_t>(status >> 8)};
            break;
        }
        case 0x6064:  // 位置实际值
        {
            int32_t position = static_cast<int32_t>(motor.position * 1000);
            data = {
                static_cast<uint8_t>(position & 0xFF),
                static_cast<uint8_t>((position >> 8) & 0xFF),
                static_cast<uint8_t>((position >> 16) & 0xFF),
                static_cast<uint8_t>((position >> 24) & 0xFF)
            };
            break;
        }
        case 0x606C:  // 速度实际值
        {
            int32_t velocity = static_cast<int32_t>(motor.velocity * 100);
            data = {
                static_cast<uint8_t>(velocity & 0xFF),
                static_cast<uint8_t>((velocity >> 8) & 0xFF),
                static_cast<uint8_t>((velocity >> 16) & 0xFF),
                static_cast<uint8_t>((velocity >> 24) & 0xFF)
            };
            break;
        }
        default:
            return false;
    }
    
    if (!data.empty()) {
        motor.object_dictionary[key] = data;
        return true;
    }
    
    return false;
}

bool VirtualMotorDevice::writeToObjectDictionary(uint16_t index, uint8_t subindex, 
                                                const std::vector<uint8_t>& data, uint8_t node_id) {
    auto& motor = motors_[node_id];
    uint32_t key = (index << 8) | subindex;
    
    motor.object_dictionary[key] = data;
    
    RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 写入对象字典 0x%04X/%d 数据长度: %zu",
                node_id, index, subindex, data.size());
    
    return true;
}

void VirtualMotorDevice::initVirtualMotors() {
    for (auto node_id : node_ids_) {
        VirtualMotorState motor;
        
        // 初始化电机参数
        motor.max_current = 20.0;
        motor.encoder_resolution = 10000;
        motor.position_kp = 10.0;
        motor.velocity_kp = 1.0;
        motor.friction_coefficient = 0.01;
        
        // 初始化状态
        motor.position = node_id * 0.1;  // 不同节点有不同的初始位置
        motor.velocity = 0.0;
        motor.torque = 0.0;
        motor.current = 0.0;
        motor.temperature = 25.0;
        motor.bus_voltage = 48.0;
        
        motor.target_position = motor.position;
        motor.target_velocity = 0.0;
        motor.target_torque = 0.0;
        
        // 初始状态
        motor.state = STATE_PRE_OP;
        motor.fault_code = 0;
        
        // 初始化对象字典中的一些关键值
        motor.object_dictionary[(0x6040 << 8) | 0x00] = {0x00, 0x00};  // 控制字
        motor.object_dictionary[(0x6060 << 8) | 0x00] = {0x08};        // 工作模式
        
        motors_[node_id] = motor;
        
        RCLCPP_INFO(this->get_logger(), 
                   "初始化节点 0x%02X: 位置=%.3f, 最大速度=%.1f RPM", 
                   node_id, motor.position, max_speed_);
    }
}

void VirtualMotorDevice::simulateAllMotors() {
    for (auto& pair : motors_) {
        uint8_t node_id = pair.first;
        VirtualMotorState& motor = pair.second;
        
        simulateMotor(node_id, motor);
    }
}

void VirtualMotorDevice::simulateMotor(uint8_t node_id, VirtualMotorState& motor) {
    if (motor.state == STATE_OPERATIONAL && motor.fault_code == 0) {
        // 模拟电机物理行为
        double position_error = motor.target_position - motor.position;
        if (fabs(position_error) > 0.001) {
            motor.velocity = position_error * motor.position_kp;
            motor.velocity = std::clamp(motor.velocity, -max_speed_, max_speed_);
        }
        
        // 速度积分计算位置
        motor.position += motor.velocity * 0.01;  // 10ms周期
        
        // 模拟摩擦
        motor.torque = motor.velocity * motor.friction_coefficient;
        motor.current = motor.torque / 0.5;
        
        // 模拟温度变化
        motor.temperature = 25.0 + fabs(motor.torque * motor.velocity) * 0.001;
        motor.temperature = std::min(motor.temperature, 80.0);
        
        // 检查故障
        checkFaults(node_id, motor);
        
        // 定期发送PDO
        static std::map<uint8_t, int> pdo_counters;
        int& counter = pdo_counters[node_id];
        counter++;
        
        // if (counter % 10 == 0) {  // 每100ms发送一次
        //     sendTPDO1(node_id, motor);
        // }
        // if (counter % 20 == 0) {  // 每200ms发送一次
        //     sendTPDO2(node_id, motor);
        // }
    }
}

void VirtualMotorDevice::sendTPDO1(uint8_t node_id, VirtualMotorState& motor) {
    uint16_t status = getStatusWord(node_id);
    int32_t position = static_cast<int32_t>(motor.position * 1000);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(status & 0xFF),
        static_cast<uint8_t>(status >> 8),
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>((position >> 16) & 0xFF),
        static_cast<uint8_t>((position >> 24) & 0xFF)
    };
    
    // sendCANFrame(canopen::PDO1_TX_BASE_ID + node_id, data);
    
    // RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 发送TPDO1, 位置=%.3f", node_id, motor.position);
}

void VirtualMotorDevice::sendTPDO2(uint8_t node_id, VirtualMotorState& motor) {
    int32_t velocity = static_cast<int32_t>(motor.velocity * 100);
    int16_t torque = static_cast<int16_t>(motor.torque * 1000);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF),
        static_cast<uint8_t>(torque & 0xFF),
        static_cast<uint8_t>((torque >> 8) & 0xFF)
    };
    
    sendCANFrame(canopen::PDO2_TX_BASE_ID + node_id, data);
    
    RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 发送TPDO2, 速度=%.1f", node_id, motor.velocity);
}

void VirtualMotorDevice::processRPDO1(const struct can_frame& frame, uint8_t node_id) {
    if (frame.can_dlc >= 6) {
        auto& motor = motors_[node_id];
        
        uint16_t control_word = (frame.data[1] << 8) | frame.data[0];
        int32_t target_pos = (frame.data[5] << 24) | (frame.data[4] << 16) | 
                            (frame.data[3] << 8) | frame.data[2];
        
        motor.target_position = target_pos / 1000.0;
        processControlWord(control_word, node_id);
        
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 收到RPDO1, 目标位置=%.3f", 
                   node_id, motor.target_position);
    }
}

void VirtualMotorDevice::processRPDO2(const struct can_frame& frame, uint8_t node_id) {
    if (frame.can_dlc >= 4) {
        auto& motor = motors_[node_id];
        
        int32_t target_vel = (frame.data[3] << 24) | (frame.data[2] << 16) | 
                            (frame.data[1] << 8) | frame.data[0];
        
        motor.target_velocity = target_vel / 100.0;
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 收到RPDO2, 目标速度=%.1f", 
                   node_id, motor.target_velocity);
    }
}

uint16_t VirtualMotorDevice::getStatusWord(uint8_t node_id) {
    const auto& motor = motors_[node_id];
    uint16_t status = 0;
    
    switch (motor.state) {
        case STATE_PRE_OP:
            status = 0x0121;  // 准备上电
            break;
        case STATE_SWITCHED_ON:
            status = 0x0233;  // 已上电
            break;
        case STATE_OPERATIONAL:
            status = 0x0237;  // 运行使能
            if (motor.velocity != 0) {
                status |= 0x4000;  // 运动中
            }
            break;
        case STATE_STOPPED:
            status = 0x0221;  // 已停止
            break;
        case STATE_FAULT:
            status = 0x0008;  // 故障
            break;
    }
    
    return status;
}

void VirtualMotorDevice::processControlWord(uint16_t control_word, uint8_t node_id) {
    auto& motor = motors_[node_id];
    
    bool switch_on = (control_word & 0x0007) == 0x0007;
    bool enable_op = (control_word & 0x000F) == 0x000F;
    bool fault_reset = (control_word & 0x0080) == 0x0080;
    
    if (fault_reset && motor.fault_code != 0) {
        motor.fault_code = 0;
        motor.state = STATE_PRE_OP;
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 故障复位完成", node_id);
    }
    
    if (enable_op) {
        motor.state = STATE_OPERATIONAL;
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 运行使能", node_id);
    } else if (switch_on) {
        motor.state = STATE_SWITCHED_ON;
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 电机上电", node_id);
    }
}

void VirtualMotorDevice::checkFaults(uint8_t node_id, VirtualMotorState& motor) {
    if (motor.temperature > 75.0 && motor.fault_code == 0) {
        motor.fault_code = 0x1000;  // 过温
        motor.state = STATE_FAULT;
        RCLCPP_ERROR(this->get_logger(), "节点 0x%02X: 过温故障 (%.1f°C)", node_id, motor.temperature);
        
        // 发送紧急报文
        std::vector<uint8_t> emcy = {
            0x10, 0x00,  // 错误码
            0x00,        // 错误寄存器
            0x00, 0x00, 0x00, 0x00, 0x00  // 制造商特定
        };
        sendCANFrame(canopen::EMCY_BASE_ID + node_id, emcy);
    }
    
    if (fabs(motor.current) > motor.max_current && motor.fault_code == 0) {
        motor.fault_code = 0x2000;  // 过流
        motor.state = STATE_FAULT;
        RCLCPP_ERROR(this->get_logger(), "节点 0x%02X: 过流故障 (%.1fA)", node_id, motor.current);
    }
    
    if (fabs(motor.velocity) > max_speed_ * 1.1 && motor.fault_code == 0) {
        motor.fault_code = 0x3000;  // 超速
        motor.state = STATE_FAULT;
        RCLCPP_ERROR(this->get_logger(), "节点 0x%02X: 超速故障 (%.1f RPM)", node_id, motor.velocity);
    }
}

void VirtualMotorDevice::publishAllJointStates() {
    for (auto& pair : motors_) {
        uint8_t node_id = pair.first;
        const VirtualMotorState& motor = pair.second;
        
        if (joint_state_pubs_.find(node_id) != joint_state_pubs_.end() &&
            joint_state_pubs_[node_id]->get_subscription_count() > 0) {
            
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = this->now();
            msg.header.frame_id = "virtual_motor_node_" + std::to_string(node_id);
            msg.name.push_back("motor_joint");
            msg.position.push_back(motor.position);
            msg.velocity.push_back(motor.velocity);
            msg.effort.push_back(motor.torque);
            
            joint_state_pubs_[node_id]->publish(msg);
        }
    }
}

void VirtualMotorDevice::handleEnableService(
    uint8_t node_id,
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
    
    auto& motor = motors_[node_id];
    
    if (request->data) {
        if (motor.state == STATE_FAULT) {
            response->success = false;
            response->message = "电机处于故障状态，请先复位";
            RCLCPP_WARN(this->get_logger(), "节点 0x%02X: 尝试使能处于故障状态的电机", node_id);
        } else {
            // 模拟通过CANopen使能
            processControlWord(0x000F, node_id);
            response->success = true;
            response->message = "电机已通过CANopen使能";
            RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 通过服务使能电机", node_id);
        }
    } else {
        processControlWord(0x0006, node_id);
        response->success = true;
        response->message = "电机已通过CANopen停止";
        RCLCPP_INFO(this->get_logger(), "节点 0x%02X: 通过服务停止电机", node_id);
    }
}