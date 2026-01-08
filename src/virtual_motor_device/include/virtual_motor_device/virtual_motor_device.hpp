// include/virtual_motor_device/virtual_motor_device.hpp
#ifndef VIRTUAL_MOTOR_DEVICE_HPP
#define VIRTUAL_MOTOR_DEVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

using namespace std::chrono_literals;

// CANopen相关定义
namespace canopen {
    enum class NmtState {
        BOOTUP = 0x00,
        STOPPED = 0x04,
        OPERATIONAL = 0x05,
        PRE_OPERATIONAL = 0x7F
    };

    // CANopen帧ID定义
    const uint32_t NMT_ID = 0x000;           // NMT帧
    const uint32_t SYNC_ID = 0x080;          // SYNC帧
    const uint32_t TIME_ID = 0x100;          // TIME帧
    const uint32_t EMCY_BASE_ID = 0x080;     // 紧急帧基础ID
    const uint32_t PDO1_TX_BASE_ID = 0x180;  // TPDO1基础ID
    const uint32_t PDO1_RX_BASE_ID = 0x200;  // RPDO1基础ID
    const uint32_t PDO2_TX_BASE_ID = 0x280;  // TPDO2基础ID
    const uint32_t PDO2_RX_BASE_ID = 0x300;  // RPDO2基础ID
    const uint32_t SDO_TX_BASE_ID = 0x580;   // SDO Tx基础ID
    const uint32_t SDO_RX_BASE_ID = 0x600;   // SDO Rx基础ID
    
    // NMT命令
    enum NmtCommand : uint8_t {
        NMT_START = 0x01,
        NMT_STOP = 0x02,
        NMT_ENTER_PREOP = 0x80,
        NMT_RESET_NODE = 0x81,
        NMT_RESET_COMM = 0x82
    };
    
    // SDO命令
    enum SdoCommand : uint8_t {
        SDO_UPLOAD_INITIATE = 0x40,      // 上传请求
        SDO_UPLOAD_SEGMENT = 0x60,       // 上传分段
        SDO_DOWNLOAD_INITIATE = 0x20,    // 下载请求
        SDO_DOWNLOAD_SEGMENT = 0x00,     // 下载分段
        SDO_ABORT = 0x80                 // 中止传输
    };
}

class VirtualMotorDevice : public rclcpp::Node {
public:
    explicit VirtualMotorDevice(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~VirtualMotorDevice();
    
private:
    // CAN接口相关
    bool initCANInterface(const std::string& interface = "vcan0");
    void canReadThread();
    void canWriteThread();
    void sendCANFrame(uint32_t can_id, const std::vector<uint8_t>& data);
    bool receiveCANFrame(struct can_frame& frame);
    
    // CANopen协议处理
    void processCANFrame(const struct can_frame& frame);
    void processNMTFrame(const struct can_frame& frame);
    void processSDOFrame(const struct can_frame& frame);
    void processPDOFrame(const struct can_frame& frame);
    
    // SDO处理函数
    void handleSDORead(uint16_t index, uint8_t subindex, uint8_t node_id);
    void handleSDOWrite(uint16_t index, uint8_t subindex, const std::vector<uint8_t>& data, uint8_t node_id);
    void sendSDOResponse(uint16_t index, uint8_t subindex, const std::vector<uint8_t>& data, uint8_t node_id, bool success = true);
    
    // PDO处理
    void sendTPDO1();
    void sendTPDO2();
    void processRPDO1(const struct can_frame& frame);
    void processRPDO2(const struct can_frame& frame);
    
    // 对象字典操作
    bool readFromObjectDictionary(uint16_t index, uint8_t subindex, std::vector<uint8_t>& data);
    bool writeToObjectDictionary(uint16_t index, uint8_t subindex, const std::vector<uint8_t>& data);
    
    // 电机模拟
    void initVirtualMotor();
    void simulateMotor();
    void publishJointState();
    
    // 状态字和控制字处理
    uint16_t getStatusWord();
    void processControlWord(uint16_t control_word);
    void checkFaults();
    
    // ROS2服务回调
    void handleEnableService(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response);
    
    // 电机状态枚举
    enum MotorState {
        STATE_NOT_READY,
        STATE_BOOTUP,
        STATE_PRE_OP,
        STATE_SWITCHED_ON,
        STATE_OPERATIONAL,
        STATE_STOPPED,
        STATE_FAULT
    };
    
    // CAN相关成员
    int can_socket_;
    std::atomic<bool> can_running_;
    std::thread can_read_thread_;
    std::thread can_write_thread_;
    std::mutex can_mutex_;
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> can_write_queue_;
    
    // 对象字典存储
    std::unordered_map<uint32_t, std::vector<uint8_t>> object_dictionary_;
    
    // 电机参数
    uint8_t node_id_;
    double max_speed_;
    double max_torque_;
    double max_current_;
    int encoder_resolution_;
    
    // 状态变量
    double position_;
    double velocity_;
    double torque_;
    double current_;
    double temperature_;
    double bus_voltage_;
    
    // 目标值
    double target_position_;
    double target_velocity_;
    double target_torque_;
    
    // 控制参数
    double position_kp_;
    double velocity_kp_;
    double friction_coefficient_;
    
    MotorState state_;
    uint16_t fault_code_;
    
    // ROS2组件
    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
};

#endif // VIRTUAL_MOTOR_DEVICE_HPP