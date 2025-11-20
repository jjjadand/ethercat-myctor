// MTDeviceController.h
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <ecrt.h>   // IgH EtherCAT master API

class MTDeviceController {
public:
    void setExternalRunningFlag(std::atomic<bool>* flag);

    // 运行时状态快照（按对象字典类型）
    struct Status {
        uint16_t status_word = 0;          // 0x6041 U16
        int32_t  actual_position = 0;      // 0x6064 S32
        int32_t  actual_velocity = 0;      // 0x606C S32
        int16_t  actual_torque = 0;        // 0x6077 S16
        uint16_t error_code = 0;           // 0x603F U16
        int8_t   operation_mode_display = 0; // 0x6061 S8

        bool     is_enabled = false;
        bool     is_fault = false;
        bool     target_reached = false;
    };

    // PDO 条目的域内字节偏移（注意：偏移可能为 0，不能用 0 判断未映射）
    // struct MTDeviceOffsets {
    //     // RxPDO (Master -> Slave)
    //     uint32_t control_word;            // 0x6040 U16
    //     uint32_t operation_mode;          // 0x6060 S8
    //     uint32_t target_position;         // 0x607A S32
    //     uint32_t target_velocity;         // 0x60FF S32
    //     uint32_t target_torque;           // 0x6071 S16
    //     uint32_t max_torque;              // 0x6072 U16

    //     // TxPDO (Slave -> Master)
    //     uint32_t status_word;             // 0x6041 U16
    //     uint32_t operation_mode_display;  // 0x6061 S8
    //     uint32_t actual_position;         // 0x6064 S32
    //     uint32_t actual_velocity;         // 0x606C S32
    //     uint32_t actual_torque;           // 0x6077 S16
    //     uint32_t error_code;              // 0x603F U16
    // };

    struct MTDeviceOffsets {
    // Rx PDO (Master -> Slave)
    uint32_t control_word;        // 0x6040 (16-bit)
    uint32_t target_position;     // 0x607A (32-bit)
    uint32_t target_velocity;     // 0x60FF (32-bit) - optional
    uint32_t target_torque;       // 0x6071 (16-bit)
    uint32_t max_torque;          // 0x6072 (16-bit)
    uint32_t operation_mode;      // 0x6060 (8-bit)
    uint32_t dummy_rx_5ffe;       // 0x5FFE (8-bit) 或其他占位

    // Tx PDO (Slave -> Master)
    uint32_t status_word;         // 0x6041 (16-bit)
    uint32_t actual_position;     // 0x6064 (32-bit)
    uint32_t actual_velocity;     // 0x606C (32-bit)
    uint32_t actual_torque;       // 0x6077 (16-bit)
    uint32_t error_code;          // 0x603F (16-bit)
    uint32_t operation_mode_display; // 0x6061 (8-bit)
    uint32_t dummy_tx_5ffe;       // 0x5FFE (8-bit) 或其他占位

    };

public:
    MTDeviceController();
    ~MTDeviceController();

    // 初始化并注册现有 PDO 映射（不修改从站映射）
    // ifname 例如 "eno1"；slave_pos 从 0 开始
    bool init(const char* ifname, int slave_pos);

    // 安全关闭（停止实时线程、下电、释放 master）
    void shutdown();

    // 上电到 Operation Enabled（CiA-402 典型序列）
    bool enable();

    // 下电（宽松等待 ReadyToSwitchOn）
    bool disable();

    // 设置模式（示例：CSP = 0x08；若要 PP=1 / PV=3 请改 .cpp 中写入的值）
    bool setPositionMode();
    bool waitForModeChange(int8_t expected_mode, uint32_t timeout_ms);
    // 设定目标位置（S32）
    bool setTargetPosition(int32_t pos);

    // 启停实时循环（周期内完成 receive/process/queue/send + 用户回调）
    bool startRealtimeThread(uint32_t freq_hz);
    void stopRealtimeThread();

    // 获取当前状态快照（从 PDO 中读）
    Status getStatus();

    // 注册控制回调：在实时循环内回调，传入最新 Status，可在其中写目标或控制字
    void setControlCallback(std::function<void(Status&)> cb);

private:
    // 实时线程函数
    void realtimeThreadFunc(uint32_t freq_hz);

    // 等待状态字达到期望掩码（按 CiA-402 位判断）
    bool waitForState(uint16_t expect_mask, uint32_t timeout_ms = 1000);

    // 等待域开始交换（working_counter > 0）
    bool waitForWorkingCounter(uint32_t timeout_ms);

    // 写控制字（带映射检测）
    bool writeControlWord(uint16_t val);

    // 读状态字（未映射则返回 0）
    uint16_t readStatusWord() const;

    // 偏移是否有效（非“未映射”哨兵）
    bool isMapped(uint32_t off) const;

    // 全量检查 offsets 是否都已映射（初始化后调用）
    bool verifyOffsetsMapped() const;

private:
    // EtherCAT master / domain / slave config
    ec_master_t*        master_;
    ec_domain_t*        domain_;
    ec_slave_config_t*  slave_config_;
    uint8_t*            domain_pd_;

    ec_master_state_t   master_state_;
    ec_domain_state_t   domain_state_;

    // PDO 偏移集合（以字节为单位）
    MTDeviceOffsets     offsets_;

    // 用户控制回调（在实时周期内调用）
    std::function<void(Status&)> control_cb_;

    // 实时线程
    std::atomic<bool>   running_{false};
    std::atomic<bool>* external_running_ = nullptr; // 新增
    std::thread         realtime_thread_;
    std::mutex          mutex_;
};

