// include/virtual_motor_device/virtual_motor_device.hpp
#ifndef VIRTUAL_MOTOR_DEVICE__VIRTUAL_MOTOR_DEVICE_HPP_
#define VIRTUAL_MOTOR_DEVICE__VIRTUAL_MOTOR_DEVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

#include <thread>
#include <mutex>
#include <vector>
#include <map>
#include <string>
#include <functional>

#include "virtual_motor_device/canopen_definitions.hpp"

class VirtualMotorDevice : public rclcpp::Node {
public:
    explicit VirtualMotorDevice(const rclcpp::NodeOptions & options);
    virtual ~VirtualMotorDevice();

private:
    // 电机状态结构体
    struct VirtualMotorState {
        double position = 0.0;
        double velocity = 0.0;
        double torque = 0.0;
        double current = 0.0;
        double temperature = 25.0;
        double bus_voltage = 48.0;
        
        double target_position = 0.0;
        double target_velocity = 0.0;
        double target_torque = 0.0;
        
        double max_current = 20.0;
        int encoder_resolution = 10000;
        double position_kp = 10.0;
        double velocity_kp = 1.0;
        double friction_coefficient = 0.01;
        
        uint8_t state = 0;
        uint16_t fault_code = 0;
        
        std::map<uint32_t, std::vector<uint8_t>> object_dictionary;
    };

    // CAN相关
    int can_socket_;
    bool can_running_;
    std::thread can_read_thread_;
    std::thread can_write_thread_;
    std::mutex can_mutex_;
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> can_write_queue_;
    
    // 电机相关
    std::vector<uint8_t> node_ids_;  // 所有节点ID
    std::map<uint8_t, VirtualMotorState> motors_;  // 每个节点的电机状态
    double max_speed_;
    double max_torque_;
    
    // ROS2相关
    std::map<uint8_t, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> joint_state_pubs_;
    std::map<uint8_t, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> enable_services_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // CAN接口初始化
    bool initCANInterface(const std::string& interface);
    void canReadThread();
    void canWriteThread();
    void sendCANFrame(uint32_t can_id, const std::vector<uint8_t>& data);
    bool receiveCANFrame(struct can_frame& frame);
    void processCANFrame(const struct can_frame& frame);
    
    // NMT处理
    void processNMTFrame(const struct can_frame& frame);
    void handleNMTCommand(uint8_t command, uint8_t node_id);
    
    // SDO处理
    void processSDOFrame(const struct can_frame& frame, uint8_t node_id);
    void handleSDORead(uint16_t index, uint8_t subindex, uint8_t node_id);
    void handleSDOWrite(uint16_t index, uint8_t subindex, 
                       const std::vector<uint8_t>& data, uint8_t node_id);
    void sendSDOResponse(uint16_t index, uint8_t subindex,
                        const std::vector<uint8_t>& data, uint8_t node_id, bool success);
    
    // PDO处理
    void processRPDO1(const struct can_frame& frame, uint8_t node_id);
    void processRPDO2(const struct can_frame& frame, uint8_t node_id);
    void sendTPDO1(uint8_t node_id, VirtualMotorState& motor);
    void sendTPDO2(uint8_t node_id, VirtualMotorState& motor);
    
    // 对象字典操作
    bool readFromObjectDictionary(uint16_t index, uint8_t subindex, 
                                 std::vector<uint8_t>& data, uint8_t node_id);
    bool writeToObjectDictionary(uint16_t index, uint8_t subindex, 
                                const std::vector<uint8_t>& data, uint8_t node_id);
    
    // 电机模拟
    void initVirtualMotors();
    void simulateAllMotors();
    void simulateMotor(uint8_t node_id, VirtualMotorState& motor);
    uint16_t getStatusWord(uint8_t node_id);
    void processControlWord(uint16_t control_word, uint8_t node_id);
    void checkFaults(uint8_t node_id, VirtualMotorState& motor);
    
    // ROS2发布
    void publishAllJointStates();
    
    // 服务处理
    void handleEnableService(uint8_t node_id,
                           const std_srvs::srv::SetBool::Request::SharedPtr request,
                           std_srvs::srv::SetBool::Response::SharedPtr response);
};

#endif  // VIRTUAL_MOTOR_DEVICE__VIRTUAL_MOTOR_DEVICE_HPP_