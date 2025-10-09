// MTDeviceDriver.h
#ifndef MT_DEVICE_DRIVER_H
#define MT_DEVICE_DRIVER_H

#include <ecrt.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>

class MTDeviceDriver {
public:
    MTDeviceDriver();
    ~MTDeviceDriver();
    
    // 初始化函数
    bool init(const char* interface = "eth0", int slave_position = 0);
    void shutdown();
    
    // 状态控制
    bool enable();
    bool disable();
    bool resetFault();
    
    // 运动控制
    bool setPositionMode();
    bool setVelocityMode();
    bool setTorqueMode();
    
    bool setTargetPosition(int32_t position);
    bool setTargetVelocity(int32_t velocity);
    bool setTargetTorque(int16_t torque);
    bool setMaxTorque(uint16_t max_torque);
    
    // 状态读取
    struct Status {
        uint16_t control_word;
        uint16_t status_word;
        int32_t actual_position;
        int32_t actual_velocity;
        int16_t actual_torque;
        uint16_t error_code;
        int8_t operation_mode;
        int8_t operation_mode_display;
        bool is_enabled;
        bool is_fault;
        bool target_reached;
    };
    
    Status getStatus();
    
    // 实时控制循环
    bool startRealtimeThread(uint32_t frequency_hz = 1000);
    void stopRealtimeThread();
    void setControlCallback(std::function<void(Status&)> callback);

private:
    // EtherCAT 相关
    ec_master_t* master_;
    ec_master_state_t master_state_;
    ec_domain_t* domain_;
    ec_domain_state_t domain_state_;
    ec_slave_config_t* slave_config_;
    
    // PDO 条目偏移量
    // struct PDOOffsets {
    //     uint16_t control_word;
    //     uint16_t status_word;
    //     uint16_t target_position;
    //     uint16_t actual_position;
    //     uint16_t target_velocity;
    //     uint16_t actual_velocity;
    //     uint16_t target_torque;
    //     uint16_t actual_torque;
    //     uint16_t max_torque;
    //     uint16_t operation_mode;
    //     uint16_t operation_mode_display;
    //     uint16_t error_code;
    // } offsets_;
        struct PDOOffsets {
        unsigned int control_word;
        unsigned int status_word;
        unsigned int target_position;
        unsigned int actual_position;
        unsigned int target_velocity;
        unsigned int actual_velocity;
        unsigned int target_torque;
        unsigned int actual_torque;
        unsigned int max_torque;
        unsigned int operation_mode;
        unsigned int operation_mode_display;
        unsigned int error_code;
    } offsets_;
    
    // 域数据指针
    uint8_t* domain_pd_;
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread realtime_thread_;
    std::function<void(Status&)> control_callback_;
    
    // 状态机辅助函数
    bool waitForState(uint16_t expected_state, uint32_t timeout_ms = 5000);
    bool writeControlWord(uint16_t value);
    uint16_t readStatusWord();
    
    // 实时线程函数
    void realtimeThreadFunc(uint32_t frequency_hz);
    
    mutable std::mutex mutex_;
};

#endif // MT_DEVICE_DRIVER_H
