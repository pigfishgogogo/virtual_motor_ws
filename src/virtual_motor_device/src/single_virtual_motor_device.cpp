// src/virtual_motor_device.cpp
#include "virtual_motor_device/single_virtual_motor_device.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

VirtualMotorDevice::VirtualMotorDevice(const rclcpp::NodeOptions & options)
: rclcpp::Node("virtual_motor_device", options),
  can_socket_(-1),
  can_running_(false)
{
    RCLCPP_INFO(this->get_logger(), "虚拟电机CANopen设备启动");
    
    // 从参数服务器读取参数
    this->declare_parameter<uint8_t>("node_id", 0x01);
    this->declare_parameter<std::string>("can_interface", "vcan0");
    this->declare_parameter<double>("max_speed", 3000.0);
    this->declare_parameter<double>("max_torque", 10.0);
    
    node_id_ = this->get_parameter("node_id").as_int();
    std::string can_interface = this->get_parameter("can_interface").as_string();
    max_speed_ = this->get_parameter("max_speed").as_double();
    max_torque_ = this->get_parameter("max_torque").as_double();
    
    RCLCPP_INFO(this->get_logger(), "节点ID: 0x%02X, CAN接口: %s", node_id_, can_interface.c_str());
    
    // 初始化CAN接口
    if (!initCANInterface(can_interface)) {
        RCLCPP_FATAL(this->get_logger(), "CAN接口初始化失败");
        return;
    }
    
    // 初始化对象字典和电机参数
    initVirtualMotor();
    
    // 创建ROS2发布器
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/virtual_motor/joint_states", 10);
    
    // 创建服务
    enable_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/virtual_motor/enable",
        std::bind(&VirtualMotorDevice::handleEnableService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // 启动CAN读写线程
    can_running_ = true;
    can_read_thread_ = std::thread(&VirtualMotorDevice::canReadThread, this);
    can_write_thread_ = std::thread(&VirtualMotorDevice::canWriteThread, this);
    
    // 创建定时器
    sim_timer_ = this->create_wall_timer(
        500ms, std::bind(&VirtualMotorDevice::simulateMotor, this)); // 10 -> 500ms
    
    publish_timer_ = this->create_wall_timer(
        500ms, std::bind(&VirtualMotorDevice::publishJointState, this)); // 20ms -》 500ms
    
    // 发送启动报文（心跳）
    sendCANFrame(canopen::NMT_ID, {0x01, node_id_});  // NMT启动命令
    RCLCPP_INFO(this->get_logger(), "发送NMT启动命令");
    
    RCLCPP_INFO(this->get_logger(), "虚拟电机CANopen设备初始化完成");
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
    // 监听主站发送的命令  如监听cansend 601#2f 6060 00 03 00 00 00
    struct can_frame frame;
    struct pollfd fds[1];
    
    fds[0].fd = can_socket_;
    fds[0].events = POLLIN;
    
    while (can_running_) {
        int ret = poll(fds, 1, 100);  // 100ms超时 ret ==0超时、 >0有事件发生、 <0发生错误
        
        if (ret > 0 && (fds[0].revents & POLLIN)) { 
            if (receiveCANFrame(frame)) {  // 从socket中 read()  得到 can_frame类型的frame数据
                // 处理frame数据，先根据can_id分发是SDO还是PDO处理，再进入内部，如果是SDO,根据command分为 上传/下载  
                // 上传和下载都是对 （对象字典） object_dictionary_ 进行修改，这个object_dictionary_就是8位数据帧结构
                // 上传和下载除了操作 对象字典， 还会通过 sendCANFrame 进行响应是否上传成功或下载成功，以及模拟修改电机状态
                processCANFrame(frame);
            }
        }
    }
}

void VirtualMotorDevice::canWriteThread() {
    // 每隔10ms检查是否有frame需要send  sendCANFrame 函数作用：往can_write_queue_中追加 帧， 本线程负责系统调用write模拟电机给主站反馈帧
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
    } else if (can_id == canopen::SDO_RX_BASE_ID + node_id_) {
        processSDOFrame(frame);
    } else if (can_id == canopen::PDO1_RX_BASE_ID + node_id_) {
        processRPDO1(frame);
    } else if (can_id == canopen::PDO2_RX_BASE_ID + node_id_) {
        processRPDO2(frame);
    } else if ((can_id & 0x780) == 0x700) {  // 节点保护/心跳
        // 忽略，或实现心跳响应
    }
}

void VirtualMotorDevice::processNMTFrame(const struct can_frame& frame) {
    if (frame.can_dlc >= 2) {
        uint8_t command = frame.data[0];
        uint8_t node_id = frame.data[1];
        
        if (node_id == node_id_ || node_id == 0) {  // 0表示广播
            switch (command) {
                case canopen::NMT_START:
                    state_ = STATE_OPERATIONAL;
                    RCLCPP_INFO(this->get_logger(), "收到NMT启动命令，进入运行状态");
                    break;
                case canopen::NMT_STOP:
                    state_ = STATE_STOPPED;
                    velocity_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "收到NMT停止命令，电机停止");
                    break;
                case canopen::NMT_ENTER_PREOP:
                    state_ = STATE_PRE_OP;
                    RCLCPP_INFO(this->get_logger(), "进入预运行状态");
                    break;
            }
            
            // 发送心跳响应
            uint8_t heartbeat = static_cast<uint8_t>(state_);
            sendCANFrame(0x700 + node_id_, {heartbeat});
        }
    }
}

void VirtualMotorDevice::processSDOFrame(const struct can_frame& frame) {
    if (frame.can_dlc >= 4) {
        uint8_t command = frame.data[0];
        uint16_t index = (frame.data[2] << 8) | frame.data[1];
        uint8_t subindex = frame.data[3];
        
        RCLCPP_INFO(this->get_logger(), "收到SDO命令: 0x%02X, 索引: 0x%04X, 子索引: 0x%02X",
                   command, index, subindex);
        
        if ((command & 0xE0) == canopen::SDO_UPLOAD_INITIATE) {
            // SDO读请求
            handleSDORead(index, subindex, node_id_);
        } else if ((command & 0xE0) == canopen::SDO_DOWNLOAD_INITIATE) {
            // SDO写请求
            std::vector<uint8_t> data;
            uint8_t data_size = 4 - ((command >> 2) & 0x03);  // 从命令字中获取数据大小
            
            for (int i = 0; i < data_size && (i + 4) < frame.can_dlc; i++) {
                data.push_back(frame.data[4 + i]);
            }
            
            handleSDOWrite(index, subindex, data, node_id_);
        }
    }
}

void VirtualMotorDevice::handleSDORead(uint16_t index, uint8_t subindex, uint8_t node_id) {
    RCLCPP_INFO(this->get_logger(), "临时打印1: 处理SDO读请求 索引:0x%04X 子索引:0x%02X", index, subindex);
    std::vector<uint8_t> data;
    
    if (readFromObjectDictionary(index, subindex, data)) {
        sendSDOResponse(index, subindex, data, node_id, true);
    } else {
        // 发送错误响应
        std::vector<uint8_t> error_response = {
            canopen::SDO_ABORT,                     // 命令字
            static_cast<uint8_t>(index & 0xFF),     // 索引低字节
            static_cast<uint8_t>(index >> 8),       // 索引高字节
            subindex,                               // 子索引
            0x06, 0x00, 0x00, 0x00                  // 错误码: 对象不存在
        };
        sendCANFrame(canopen::SDO_TX_BASE_ID + node_id, error_response);
    }
}

void VirtualMotorDevice::handleSDOWrite(uint16_t index, uint8_t subindex, 
                                       const std::vector<uint8_t>& data, uint8_t node_id) {
    if (writeToObjectDictionary(index, subindex, data)) {
        // 发送成功响应
        std::vector<uint8_t> response = {
            static_cast<uint8_t>(canopen::SDO_DOWNLOAD_INITIATE | ((4 - data.size()) << 2)),
            static_cast<uint8_t>(index & 0xFF),
            static_cast<uint8_t>(index >> 8),
            subindex
        };
        
        // 复制数据（如果有）
        response.insert(response.end(), data.begin(), data.end());
        sendCANFrame(canopen::SDO_TX_BASE_ID + node_id, response);
        
        // 如果是控制字，处理它
        if (index == 0x6040 && subindex == 0) {
            if (data.size() >= 2) {
                uint16_t control_word = (data[1] << 8) | data[0];
                processControlWord(control_word);
            }
        }
    } else {
        // 发送错误响应
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
        response.push_back(static_cast<uint8_t>(canopen::SDO_UPLOAD_INITIATE | ((4 - data.size()) << 2)));
        response.push_back(static_cast<uint8_t>(index & 0xFF));
        response.push_back(static_cast<uint8_t>(index >> 8));
        response.push_back(subindex);
        response.insert(response.end(), data.begin(), data.end());
    } else {
        // 错误响应
        response = {
            canopen::SDO_ABORT,
            static_cast<uint8_t>(index & 0xFF),
            static_cast<uint8_t>(index >> 8),
            subindex,
            0x06, 0x00, 0x00, 0x00  // 对象不存在错误
        };
    }
    
    sendCANFrame(canopen::SDO_TX_BASE_ID + node_id, response);
    RCLCPP_INFO(this->get_logger(), "临时打印3: 发送SDO响应 数据内容: ");
    for (size_t i = 0; i < data.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  data[%zu] = 0x%02X", i, data[i]);
    }
}

bool VirtualMotorDevice::readFromObjectDictionary(uint16_t index, uint8_t subindex, std::vector<uint8_t>& data) {
    RCLCPP_INFO(this->get_logger(), "临时打印2: 读取对象字典 索引:0x%04X 子索引:0x%02X", index, subindex);
    uint32_t key = (index << 8) | subindex;
    
    if (object_dictionary_.find(key) != object_dictionary_.end()) {
        data = object_dictionary_[key];
        return true;
    }
    
    // 动态生成一些标准对象
    switch (index) {
        case 0x1000:  // 设备类型
            data = {0x92, 0x01, 0x02, 0x00};  // DS402伺服驱动器
            break;
        case 0x1008:  // 设备名称
            data = {'V', 'i', 'r', 't', 'u', 'a', 'l', 'M', 'o', 't', 'o', 'r'};
            break;
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
            uint16_t status = getStatusWord();
            data = {static_cast<uint8_t>(status & 0xFF), static_cast<uint8_t>(status >> 8)};
            break;
        }
        case 0x6064:  // 位置实际值
        {
            int32_t position = static_cast<int32_t>(position_ * 1000);  // 转换为微弧度
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
            int32_t velocity = static_cast<int32_t>(velocity_ * 100);  // 转换为0.01 RPM
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
        object_dictionary_[key] = data;
        return true;
    }
    
    return false;
}

bool VirtualMotorDevice::writeToObjectDictionary(uint16_t index, uint8_t subindex, const std::vector<uint8_t>& data) {
    uint32_t key = (index << 8) | subindex;
    
    // 记录日志
    std::stringstream ss;
    ss << "写入对象字典: 0x" << std::hex << index << "/" << static_cast<int>(subindex)
       << " 数据: ";
    for (auto byte : data) {
        ss << std::hex << std::setw(2) << std::setfill('0') 
           << static_cast<int>(byte) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    
    object_dictionary_[key] = data;
    return true;
}

void VirtualMotorDevice::initVirtualMotor() {
    // 初始化电机参数
    max_current_ = 20.0;
    encoder_resolution_ = 10000;
    position_kp_ = 10.0;
    velocity_kp_ = 1.0;
    friction_coefficient_ = 0.01;
    
    // 初始化状态
    position_ = 0.0;
    velocity_ = 0.0;
    torque_ = 0.0;
    current_ = 0.0;
    temperature_ = 25.0;
    bus_voltage_ = 48.0;
    
    target_position_ = 0.0;
    target_velocity_ = 0.0;
    target_torque_ = 0.0;
    
    // 初始状态
    state_ = STATE_PRE_OP;
    fault_code_ = 0;
    
    RCLCPP_INFO(this->get_logger(), 
               "虚拟电机参数: 最大速度=%.1f RPM, 最大转矩=%.1f Nm, 节点ID=0x%02X", 
               max_speed_, max_torque_, node_id_);
    
    // 初始化对象字典中的一些关键值
    writeToObjectDictionary(0x6040, 0x00, {0x00, 0x00});  // 控制字
    writeToObjectDictionary(0x6060, 0x00, {0x08});        // 工作模式
}

void VirtualMotorDevice::simulateMotor() {
    if (state_ == STATE_OPERATIONAL && fault_code_ == 0) {
        // 模拟电机物理行为
        double position_error = target_position_ - position_;
        if (fabs(position_error) > 0.001) {
            velocity_ = position_error * position_kp_;
            velocity_ = std::clamp(velocity_, -max_speed_, max_speed_);
        }
        
        // 速度积分计算位置
        position_ += velocity_ * 0.01;  // 10ms周期
        
        // 模拟摩擦
        torque_ = velocity_ * friction_coefficient_;
        current_ = torque_ / 0.5;
        
        // 模拟温度变化
        temperature_ = 25.0 + fabs(torque_ * velocity_) * 0.001;
        temperature_ = std::min(temperature_, 80.0);
        
        // 检查故障
        checkFaults();
        
        // 定期发送PDO
        static int pdo_counter = 0;
        if (pdo_counter++ % 10 == 0) {  // 每100ms发送一次
            sendTPDO1();
        }
        if (pdo_counter % 20 == 0) {  // 每200ms发送一次
            sendTPDO2();
        }
    }
}

void VirtualMotorDevice::sendTPDO1() {
    // TPDO1: 状态字(0x6041) + 位置实际值(0x6064)
    uint16_t status = getStatusWord();
    int32_t position = static_cast<int32_t>(position_ * 1000);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(status & 0xFF),
        static_cast<uint8_t>(status >> 8),
        static_cast<uint8_t>(position & 0xFF),
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>((position >> 16) & 0xFF),
        static_cast<uint8_t>((position >> 24) & 0xFF)
    };
    
    sendCANFrame(canopen::PDO1_TX_BASE_ID + node_id_, data);
    RCLCPP_INFO(this->get_logger(), "发送TPDO1: 状态=0x%04X, 位置=%d", status, position);
}

void VirtualMotorDevice::sendTPDO2() {
    // TPDO2: 速度实际值(0x606C) + 转矩实际值(0x6077)
    int32_t velocity = static_cast<int32_t>(velocity_ * 100);
    int16_t torque = static_cast<int16_t>(torque_ * 1000);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(velocity & 0xFF),
        static_cast<uint8_t>((velocity >> 8) & 0xFF),
        static_cast<uint8_t>((velocity >> 16) & 0xFF),
        static_cast<uint8_t>((velocity >> 24) & 0xFF),
        static_cast<uint8_t>(torque & 0xFF),
        static_cast<uint8_t>((torque >> 8) & 0xFF)
    };
    
    sendCANFrame(canopen::PDO2_TX_BASE_ID + node_id_, data);
    RCLCPP_INFO(this->get_logger(), "发送TPDO2: 速度=%.1f, 转矩=%.2f", velocity_, torque_);
}

void VirtualMotorDevice::processRPDO1(const struct can_frame& frame) {
    if (frame.can_dlc >= 6) {
        // RPDO1: 控制字(0x6040) + 目标位置(0x607A)
        uint16_t control_word = (frame.data[1] << 8) | frame.data[0];
        int32_t target_pos = (frame.data[5] << 24) | (frame.data[4] << 16) | 
                            (frame.data[3] << 8) | frame.data[2];
        
        target_position_ = target_pos / 1000.0;
        processControlWord(control_word);
        
        RCLCPP_INFO(this->get_logger(), "收到RPDO1: 控制字=0x%04X, 目标位置=%.3f",
                   control_word, target_position_);
    }
}

void VirtualMotorDevice::processRPDO2(const struct can_frame& frame) {
    if (frame.can_dlc >= 4) {
        // RPDO2: 目标速度(0x60FF)
        int32_t target_vel = (frame.data[3] << 24) | (frame.data[2] << 16) | 
                            (frame.data[1] << 8) | frame.data[0];
        
        target_velocity_ = target_vel / 100.0;
        RCLCPP_INFO(this->get_logger(), "收到RPDO2: 目标速度=%.1f RPM", target_velocity_);
    }
}

uint16_t VirtualMotorDevice::getStatusWord() {
    uint16_t status = 0;
    
    switch (state_) {
        case STATE_PRE_OP:
            status = 0x0121;  // 准备上电
            break;
        case STATE_SWITCHED_ON:
            status = 0x0233;  // 已上电
            break;
        case STATE_OPERATIONAL:
            status = 0x0237;  // 运行使能
            if (velocity_ != 0) {
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

void VirtualMotorDevice::processControlWord(uint16_t control_word) {
    bool switch_on = (control_word & 0x0007) == 0x0007;
    bool enable_op = (control_word & 0x000F) == 0x000F;
    bool fault_reset = (control_word & 0x0080) == 0x0080;
    
    if (fault_reset && fault_code_ != 0) {
        fault_code_ = 0;
        state_ = STATE_PRE_OP;
        RCLCPP_INFO(this->get_logger(), "故障复位完成");
    }
    
    if (enable_op) {
        state_ = STATE_OPERATIONAL;
        RCLCPP_INFO(this->get_logger(), "运行使能");
    } else if (switch_on) {
        state_ = STATE_SWITCHED_ON;
        RCLCPP_INFO(this->get_logger(), "电机上电");
    }
}

void VirtualMotorDevice::checkFaults() {
    if (temperature_ > 75.0 && fault_code_ == 0) {
        fault_code_ = 0x1000;  // 过温
        state_ = STATE_FAULT;
        RCLCPP_ERROR(this->get_logger(), "故障: 过温 (%.1f°C)", temperature_);
        
        // 发送紧急报文
        std::vector<uint8_t> emcy = {
            0x10, 0x00,  // 错误码
            0x00,        // 错误寄存器
            0x00, 0x00, 0x00, 0x00, 0x00  // 制造商特定
        };
        sendCANFrame(canopen::EMCY_BASE_ID + node_id_, emcy);
    }
    
    if (fabs(current_) > max_current_ && fault_code_ == 0) {
        fault_code_ = 0x2000;  // 过流
        state_ = STATE_FAULT;
        RCLCPP_ERROR(this->get_logger(), "故障: 过流 (%.1fA)", current_);
    }
    
    if (fabs(velocity_) > max_speed_ * 1.1 && fault_code_ == 0) {
        fault_code_ = 0x3000;  // 超速
        state_ = STATE_FAULT;
        RCLCPP_ERROR(this->get_logger(), "故障: 超速 (%.1f RPM)", velocity_);
    }
}

void VirtualMotorDevice::publishJointState() {
    if (joint_state_pub_->get_subscription_count() > 0) {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "virtual_motor";
        msg.name.push_back("motor_joint");
        msg.position.push_back(position_);
        msg.velocity.push_back(velocity_);
        msg.effort.push_back(torque_);
        
        joint_state_pub_->publish(msg);
    }
}

void VirtualMotorDevice::handleEnableService(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
    
    if (request->data) {
        if (state_ == STATE_FAULT) {
            response->success = false;
            response->message = "电机处于故障状态，请先复位";
            RCLCPP_WARN(this->get_logger(), "尝试使能处于故障状态的电机");
        } else {
            // 模拟通过CANopen使能
            processControlWord(0x000F);  // 运行使能命令
            response->success = true;
            response->message = "电机已通过CANopen使能";
            RCLCPP_INFO(this->get_logger(), "通过服务使能电机");
        }
    } else {
        processControlWord(0x0006);  // 停止命令
        response->success = true;
        response->message = "电机已通过CANopen停止";
        RCLCPP_INFO(this->get_logger(), "通过服务停止电机");
    }
}