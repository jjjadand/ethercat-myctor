// MTDeviceController.cpp
#include "MTDeviceController.h"
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono;

static constexpr unsigned int INVALID_OFFSET = 0xFFFFFFFFu;

enum class EcState {
    INIT,
    SET_MODE,          // 写 6060 = 8
    ALIGN_POSITION,    // 607A = 6064
    SHUTDOWN_6,        // 6040 = 6
    SWITCH_ON_7,       // 6040 = 7
    ENABLE_OP_15,      // 6040 = 15
    RUNNING
};
EcState ec_state_ = EcState::INIT;

// ===================== 对象字典索引定义 =====================
namespace OD {
    constexpr uint16_t CONTROL_WORD             = 0x6040; // U16
    constexpr uint16_t STATUS_WORD              = 0x6041; // U16
    constexpr uint16_t OPERATION_MODE           = 0x6060; // S8
    constexpr uint16_t OPERATION_MODE_DISPLAY   = 0x6061; // S8
    constexpr uint16_t ACTUAL_POSITION          = 0x6064; // S32
    constexpr uint16_t ACTUAL_VELOCITY          = 0x606C; // S32
    constexpr uint16_t ACTUAL_TORQUE            = 0x6077; // S16
    constexpr uint16_t TARGET_POSITION          = 0x607A; // S32
    constexpr uint16_t TARGET_VELOCITY          = 0x60FF; // S32
    constexpr uint16_t TARGET_TORQUE            = 0x6071; // S16
    constexpr uint16_t MAX_TORQUE               = 0x6072; // U16
    constexpr uint16_t ERROR_CODE               = 0x603F; // U16
}

// ===================== 控制字命令（CiA-402） =====================
namespace CW {
    constexpr uint16_t SHUTDOWN          = 0x0006;
    constexpr uint16_t SWITCH_ON         = 0x0007;
    constexpr uint16_t ENABLE_OPERATION  = 0x000F;
    constexpr uint16_t QUICK_STOP        = 0x0002;
    constexpr uint16_t FAULT_RESET       = 0x0080;
}

// ===================== 状态字位（CiA-402） =====================
namespace SW {
    constexpr uint16_t READY_TO_SWITCH_ON   = 0x0001;
    constexpr uint16_t SWITCHED_ON          = 0x0002;
    constexpr uint16_t OPERATION_ENABLED    = 0x0004;
    constexpr uint16_t FAULT                = 0x0008;
    constexpr uint16_t TARGET_REACHED       = 0x0400;
    constexpr uint16_t QUICK_STOP        = 0x0020;
}

// “未映射”哨兵值：切忌用 0！
static constexpr uint32_t kInvalidOffset = 0xFFFFFFFFu;

MTDeviceController::MTDeviceController()
    : master_(nullptr),
      domain_(nullptr),
      slave_config_(nullptr),
      domain_pd_(nullptr) {
    std::memset(&master_state_, 0, sizeof(master_state_));
    std::memset(&domain_state_, 0, sizeof(domain_state_));
    std::memset(&offsets_, 0, sizeof(offsets_));
    // 将所有 offset 初始化为无效
    offsets_.control_word           = kInvalidOffset;
    offsets_.operation_mode         = kInvalidOffset;
    offsets_.target_position        = kInvalidOffset;
    offsets_.target_velocity        = kInvalidOffset;
    offsets_.target_torque          = kInvalidOffset;
    offsets_.max_torque             = kInvalidOffset;

    offsets_.status_word            = kInvalidOffset;
    offsets_.operation_mode_display = kInvalidOffset;
    offsets_.actual_position        = kInvalidOffset;
    offsets_.actual_velocity        = kInvalidOffset;
    offsets_.actual_torque          = kInvalidOffset;
    offsets_.error_code             = kInvalidOffset;
}

MTDeviceController::~MTDeviceController() {
    shutdown();
}

bool MTDeviceController::enable() {
    std::lock_guard<std::mutex> lk(mutex_);
    std::cout << "Enabling MT Device (MyActuator protocol)...\n";

    if (offsets_.control_word == INVALID_OFFSET ||
        offsets_.status_word  == INVALID_OFFSET) {
        std::cerr << "Control/Status PDO not mapped\n";
        return false;
    }

    auto do_cycle = [&](int ms = 10) {
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);
    };

    // 步骤 2：把目标位置设置成当前实际位置
    int32_t actual_pos = EC_READ_S32(domain_pd_ + offsets_.actual_position);
    EC_WRITE_S32(domain_pd_ + offsets_.target_position, actual_pos);

    // 3-1：6040 = 6  （失能）
    EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x0006);
    for (int i = 0; i < 20; ++i) do_cycle();

    // 3-2：6040 = 7  （准备使能）
    EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x0007);
    for (int i = 0; i < 20; ++i) do_cycle();

    // 3-3：6040 = 0x000F（伺服使能 + 主回路上电 + 运行）
    EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x000F);
    for (int i = 0; i < 5000; ++i) {
        do_cycle();

        uint16_t sw = EC_READ_U16(domain_pd_ + offsets_.status_word);

        // 按电机协议检查关键位：
        bool ready   = sw & (1u << 0); // Ready to switch on
        bool on      = sw & (1u << 1); // Switch on
        bool running = sw & (1u << 2); // Operation enable / running
        bool fault   = sw & (1u << 3); // Fault

        std::cout << "  SW=0x" << std::hex << sw << std::dec
                  << " ready="   << ready
                  << " on="      << on
                  << " run="     << running
                  << " fault="   << fault
                  << std::endl;

        if (fault) {
            std::cerr << "Servo reports fault via 0x6041 bit3\n";
            return false;
        }

        if (ready && on && running) {
            std::cout << "MT Device enabled (protocol 6→7→15)\n";
            return true;
        }
    }

    std::cerr << "Timeout waiting for servo enable (bits 0/1/2)\n";
    return false;
}





bool MTDeviceController::init(const char* ifname, int slave_pos) {
    std::lock_guard<std::mutex> lk(mutex_);
    std::cout << "Initializing MT Device on interface: " << ifname << "\n";

    master_ = ecrt_request_master(0);
    if (!master_) {
        std::cerr << "Failed to request EtherCAT master\n";
        return false;
    }

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        std::cerr << "Failed to create domain\n";
        ecrt_release_master(master_);
        master_ = nullptr;
        return false;
    }

    const uint32_t vendor_id  = 0x00202008u;
    const uint32_t product_id = 0x00000000u;

    slave_config_ = ecrt_master_slave_config(master_, 0, slave_pos, vendor_id, product_id);
    if (!slave_config_) {
        std::cerr << "Failed to get slave configuration\n";
        ecrt_release_master(master_);
        master_ = nullptr;
        return false;
    }


    uint16_t assign_activate = 0x0300;  // Enable DC, Sync0 active

    ecrt_slave_config_dc(
        slave_config_,
        assign_activate,
        1000000,  // 1ms 周期，和 TwinCAT 一样
        250000,        // offset_limit = 0.25ms
        0, 0      // Sync1 disabled
    );


    // -------------------------
    // 关键：按 ESI 列表**完整**注册 PDO 条目（包含 dummy / 0x5FFE）
    // 注意 index/subindex 与 bitlen（16/32/8）要和 ESI 一致
    // -------------------------
    ec_pdo_entry_reg_t regs[] = {
        // RxPDO 0x1600 (Master -> Slave)
        {0, slave_pos, vendor_id, product_id, 0x6040, 0x00, &offsets_.control_word},         // CONTROL_WORD (16)
        {0, slave_pos, vendor_id, product_id, 0x607A, 0x00, &offsets_.target_position},      // TARGET_POSITION (32)
        {0, slave_pos, vendor_id, product_id, 0x60FF, 0x00, &offsets_.target_velocity},      // TARGET_VELOCITY (32)
        {0, slave_pos, vendor_id, product_id, 0x6071, 0x00, &offsets_.target_torque},        // TARGET_TORQUE (16)
        {0, slave_pos, vendor_id, product_id, 0x6072, 0x00, &offsets_.max_torque},           // MAX_TORQUE (16)
        {0, slave_pos, vendor_id, product_id, 0x6060, 0x00, &offsets_.operation_mode},       // OPERATION_MODE (8)
        {0, slave_pos, vendor_id, product_id, 0x5FFE, 0x00, &offsets_.dummy_rx_5ffe},        // DUMMY (8) -- 保留 / 对齐
        // TxPDO 0x1A00 (Slave -> Master)
        {0, slave_pos, vendor_id, product_id, 0x6041, 0x00, &offsets_.status_word},          // STATUS_WORD (16)
        {0, slave_pos, vendor_id, product_id, 0x6064, 0x00, &offsets_.actual_position},      // ACTUAL_POSITION (32)
        {0, slave_pos, vendor_id, product_id, 0x606C, 0x00, &offsets_.actual_velocity},      // ACTUAL_VELOCITY (32)
        {0, slave_pos, vendor_id, product_id, 0x6077, 0x00, &offsets_.actual_torque},        // ACTUAL_TORQUE (16)
        {0, slave_pos, vendor_id, product_id, 0x603F, 0x00, &offsets_.error_code},           // ERROR_CODE (16)
        {0, slave_pos, vendor_id, product_id, 0x6061, 0x00, &offsets_.operation_mode_display},// OP MODE DISPLAY (8)
        {0, slave_pos, vendor_id, product_id, 0x5FFE, 0x00, &offsets_.dummy_tx_5ffe},        // DUMMY (8)
        {} // terminator
    };

    if (ecrt_domain_reg_pdo_entry_list(domain_, regs)) {
        std::cerr << "PDO entry registration failed\n";
        return false;
    }

    // 激活 master（注意：**不要**尝试使用 ecrt_slave_config_pdos 去改映射，因为设备 Fixed="true"）
    if (ecrt_master_activate(master_) != 0) {
        std::cerr << "Master activation failed\n";
        return false;
    }

    // 获取 domain 数据指针（domain size 非 0 表示映射生效）
    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_) {
        std::cerr << "Failed to get domain data pointer, domain size = "
                  << ecrt_domain_size(domain_) << std::endl;
        return false;
    }

    std::cout << "MT Device Driver initialized successfully, domain size = "
              << ecrt_domain_size(domain_) << " bytes" << std::endl;

    // ------------- 可选：在 init 时通过 SDO 预设运行模式（确保在 PREOP->SAFEOP/OP 切换时模式生效）
    // 如果你愿意用 SDO 方式在激活前写 0x6060（8 = CSP），可以启用下面一行：
    const uint8_t MODE_CSP = 0x08;
    if (ecrt_slave_config_sdo8(slave_config_, 0x6060, 0x00, MODE_CSP) != 0) {
        std::cerr << "Warning: unable to set 0x6060 via SDO (non-fatal if you will write via PDO)\n";
        // 不一定要返回 false：如果 SDO 不可用，后续我们会通过 PDO 写 0x6060 并 flush
    }

    return true;
}


void MTDeviceController::shutdown() {
    stopRealtimeThread();
    disable();

    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
    std::cout << "MT Device shutdown\n";
}

// ========== 设备上电 / 下电 ==========


bool MTDeviceController::disable() {
    std::lock_guard<std::mutex> lk(mutex_);
    std::cout << "Disabling MT Device...\n";

    if (!writeControlWord(0x0000))               return false;
    // 多数驱动关闭后会回到 Ready to Switch On 或 Switch On 之一，这里做一个宽松等待
    waitForState(SW::READY_TO_SWITCH_ON);

    std::cout << "MT Device disabled\n";
    return true;
}

// ========== 模式与目标 ==========
bool MTDeviceController::setPositionMode() {
    //std::lock_guard<std::mutex> lk(mutex_);
    
    // 1. Ensure that the operation mode PDO is mapped
    if (!isMapped(offsets_.operation_mode)) {
        std::cerr << "Operation mode PDO not mapped\n";
        return false;
    }

    // 2. Set operation mode to Position Control (CSP, 0x08)
    EC_WRITE_S8(domain_pd_ + offsets_.operation_mode, 0x08); // CSP mode
    std::cout << "Setting operation mode to CSP (0x08)\n";

    // 等待一些时间以确保模式切换生效
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 3. Read operation mode display (0x6061) to confirm mode setting
    int8_t op_mode = EC_READ_S8(domain_pd_ + offsets_.operation_mode_display);
    std::cout << "Operation mode display: " << (int)op_mode << std::endl;
    

    if (op_mode != 0x08) {
        std::cerr << "Failed to set position mode (CSP)! Current mode: " << (int)op_mode << std::endl;
        //return false;
    }else{
        std::cout << "Position mode (CSP) set successfully\n";
    }

    
    return true;
}

bool MTDeviceController::waitForModeChange(int8_t expected_mode, uint32_t timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        
        int8_t current_mode = EC_READ_S8(domain_pd_ + offsets_.operation_mode_display);
        
        if (current_mode == expected_mode) {
            std::cout << "Mode change confirmed: " << (int)current_mode << std::endl;
            return true;
        }
        
        // 继续发送模式设置
        EC_WRITE_S8(domain_pd_ + offsets_.operation_mode, expected_mode);
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cerr << "Timeout waiting for mode change to " << (int)expected_mode << std::endl;
    return false;
}

// bool MTDeviceController::setPositionMode() {
//     std::lock_guard<std::mutex> lk(mutex_);
    
//     // 检查设备状态
//     uint16_t status = readStatusWord();
//     std::cout << "Status before mode change: 0x" << std::hex << status << std::dec << std::endl;
    
//     if (!(status & SW::OPERATION_ENABLED)) {
//         std::cerr << "Device not in operation enabled state\n";
//         return false;
//     }

//     if (!isMapped(offsets_.operation_mode)) {
//         std::cerr << "Operation mode PDO not mapped\n";
//         return false;
//     }

//     // 设置模式
//     EC_WRITE_S8(domain_pd_ + offsets_.operation_mode, 0x08);
    
//     // 确保数据发送
//     ecrt_domain_queue(domain_);
//     ecrt_master_send(master_);
    
//     // 等待确认
//     return waitForModeChange(0x08, 2000); // 2秒超时
// }



bool MTDeviceController::setTargetPosition(int32_t pos) {
    std::lock_guard<std::mutex> lk(mutex_);

    EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x000F);
    EC_WRITE_S8(domain_pd_ + offsets_.operation_mode, 0x08); // CSP mode
    if (!isMapped(offsets_.target_position)) {
        std::cerr << "Target position PDO not mapped\n";
        return false;
    }
    EC_WRITE_S32(domain_pd_ + offsets_.target_position, pos);
    return true;
}

// ========== 实时循环 ==========
bool MTDeviceController::startRealtimeThread(uint32_t freq_hz) {
    if (running_.load()) {
        std::cout << "Realtime thread already running\n";
        return true;
    }
    running_.store(true);
    realtime_thread_ = std::thread(&MTDeviceController::realtimeThreadFunc, this, freq_hz);
    std::cout << "Started realtime thread at " << freq_hz << " Hz\n";
    return true;
}

void MTDeviceController::stopRealtimeThread() {
    if (!running_.load()) return;
    running_.store(false);
    if (realtime_thread_.joinable()) realtime_thread_.join();
    std::cout << "Stopped realtime thread\n";
}

// void MTDeviceController::realtimeThreadFunc(uint32_t freq_hz) {
//     const auto period = microseconds(1000000 / freq_hz);
//     auto next = steady_clock::now();
//     std::cout << "Realtime thread started\n";

//     while (running_.load()) {
//         {
//             std::lock_guard<std::mutex> lk(mutex_);
//             ecrt_master_receive(master_);
//             ecrt_domain_process(domain_);

//             if (control_cb_) {
//                 Status s = getStatus();
//                 control_cb_(s); // 用户控制回路
//             }

//             ecrt_domain_queue(domain_);
//             ecrt_master_send(master_);
//         }
//         next += period;
//         std::this_thread::sleep_until(next);
//     }

//     std::cout << "Realtime thread stopped\n";
// }


#include <time.h>
#include <sched.h>

void MTDeviceController::realtimeThreadFunc(uint32_t frequency_hz) {
    // 1. 设置线程亲和性与优先级（尝试提高实时性，防止被其他进程打断）
    struct sched_param param = {};
    param.sched_priority = 89; // 尽可能高
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    running_.store(true);
    
    const uint32_t period_ns = 1000000000 / frequency_hz; 
    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    // 状态变量
    int32_t center_pos = 0;
    int32_t current_target = 0;
    bool traj_init = false;
    bool dir = true;

    while (true) {
        if (external_running_ && !external_running_->load()) break;
        if (!running_.load()) break;

        // === 1. 精确计算下一时刻 ===
        wakeup_time.tv_nsec += period_ns;
        while (wakeup_time.tv_nsec >= 1000000000) {
            wakeup_time.tv_nsec -= 1000000000;
            wakeup_time.tv_sec++;
        }

        // === 2. 绝对睡眠 (这是 1kHz 稳定的关键) ===
        // 必须在发送 EtherCAT 数据前睡眠，确保发包间隔均匀
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

        // === 3. 设置应用时间 (DC 同步核心) ===
        // 此时醒来，时间应该是 wakeup_time 附近
        uint64_t app_time_ns = (uint64_t)wakeup_time.tv_sec * 1000000000ULL + wakeup_time.tv_nsec;
        ecrt_master_application_time(master_, app_time_ns);
        
        ecrt_master_sync_reference_clock(master_);
        ecrt_master_sync_slave_clocks(master_);

        // === 4. 收发数据 ===
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);

        // 检查域状态 
        ecrt_domain_state(domain_, &domain_state_);
        
        // 读取数据
        uint16_t sw = 0;
        int32_t pos_act = 0;
        if (isMapped(offsets_.status_word)) sw = EC_READ_U16(domain_pd_ + offsets_.status_word);
        if (isMapped(offsets_.actual_position)) pos_act = EC_READ_S32(domain_pd_ + offsets_.actual_position);

        // 持续写入模式
        if (isMapped(offsets_.operation_mode)) EC_WRITE_S8(domain_pd_ + offsets_.operation_mode, 0x08);

        // 简单的状态机
        switch (ec_state_) {
            case EcState::INIT:
            case EcState::SET_MODE:
                // 初始化阶段：目标位置 = 实际位置
                if (isMapped(offsets_.target_position)) EC_WRITE_S32(domain_pd_ + offsets_.target_position, pos_act);
                if (isMapped(offsets_.control_word)) EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x0006);
                ec_state_ = EcState::SHUTDOWN_6;
                break;

            case EcState::SHUTDOWN_6:
                if (isMapped(offsets_.target_position)) EC_WRITE_S32(domain_pd_ + offsets_.target_position, pos_act);
                EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x0006);
                if ((sw & 0x021) == 0x021) ec_state_ = EcState::SWITCH_ON_7;
                break;

            case EcState::SWITCH_ON_7:
                if (isMapped(offsets_.target_position)) EC_WRITE_S32(domain_pd_ + offsets_.target_position, pos_act);
                EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x0007);
                if ((sw & 0x023) == 0x023) ec_state_ = EcState::ENABLE_OP_15;
                break;

            case EcState::ENABLE_OP_15:
                if (isMapped(offsets_.target_position)) EC_WRITE_S32(domain_pd_ + offsets_.target_position, pos_act);
                EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x000F);
                if ((sw & 0x027) == 0x027) {
                    std::cout << "Status: ENABLED. Starting motion..." << std::endl;
                    ec_state_ = EcState::RUNNING;
                    traj_init = false;
                }
                break;

            case EcState::RUNNING:
                EC_WRITE_U16(domain_pd_ + offsets_.control_word, 0x000F);
                
                if (!traj_init) {
                    center_pos = pos_act;
                    current_target = center_pos;
                    traj_init = true;
                }

                // 【关键修改】减小速度：每次只加 10 (而不是 1000)
                // 10 counts/ms = 10,000 counts/s，这非常慢且安全
                int32_t step = 10; 
                int32_t range = 50000;

                if (dir) {
                    current_target += step;
                    if (current_target > center_pos + range) dir = false;
                } else {
                    current_target -= step;
                    if (current_target < center_pos - range) dir = true;
                }

                if (isMapped(offsets_.target_position)) {
                    EC_WRITE_S32(domain_pd_ + offsets_.target_position, current_target);
                }
                
                // 打印调试 (降低频率)
                static int cnt = 0;
                if (++cnt % 1000 == 0) { // 1秒打一次
                    std::cout << "Running: Act=" << pos_act << " Tgt=" << current_target << " SW=" << std::hex << sw << std::dec << std::endl;
                }
                break;
        }

        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);
    }

    running_.store(false);
}



// ========== 状态 / 回调 ==========
MTDeviceController::Status MTDeviceController::getStatus() {
    Status s{};
    
    // 读取并打印每个字段的状态
    if (isMapped(offsets_.status_word)) {
        s.status_word = EC_READ_U16(domain_pd_ + offsets_.status_word);
        std::cout << "status_word: 0x" << std::hex << s.status_word << std::dec << std::endl;
    }
    if (isMapped(offsets_.actual_position)) {
        s.actual_position = EC_READ_S32(domain_pd_ + offsets_.actual_position);
        std::cout << "actual_position: " << s.actual_position << std::endl;
    }
    if (isMapped(offsets_.actual_velocity)) {
        s.actual_velocity = EC_READ_S32(domain_pd_ + offsets_.actual_velocity);
        std::cout << "actual_velocity: " << s.actual_velocity << std::endl;
    }
    if (isMapped(offsets_.actual_torque)) {
        s.actual_torque = EC_READ_S16(domain_pd_ + offsets_.actual_torque);
        std::cout << "actual_torque: " << s.actual_torque << std::endl;
    }
    if (isMapped(offsets_.error_code)) {
        s.error_code = EC_READ_U16(domain_pd_ + offsets_.error_code);
        std::cout << "error_code: 0x" << std::hex << s.error_code << std::dec << std::endl;
    }
    if (isMapped(offsets_.operation_mode_display)) {
        s.operation_mode_display = EC_READ_S8(domain_pd_ + offsets_.operation_mode_display);
        std::cout << "operation_mode_display: " << (int)s.operation_mode_display << std::endl;
    }

    // 打印操作模式和故障状态
    s.is_enabled = (s.status_word & SW::OPERATION_ENABLED) != 0;
    std::cout << "is_enabled: " << s.is_enabled << std::endl;

    s.is_fault = (s.status_word & SW::FAULT) != 0;
    std::cout << "is_fault: " << s.is_fault << std::endl;

    s.target_reached = (s.status_word & SW::TARGET_REACHED) != 0;
    std::cout << "target_reached: " << s.target_reached << std::endl;

    return s;
}


void MTDeviceController::setControlCallback(std::function<void(Status&)> cb) {

    control_cb_ = std::move(cb);
}

// ========== 辅助 ==========

bool MTDeviceController::waitForState(uint16_t expect_mask, uint32_t timeout_ms) {
    auto t0 = steady_clock::now();
    int dbg_count = 0;

    while (duration_cast<milliseconds>(steady_clock::now() - t0).count() < timeout_ms) {
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);

        uint16_t sw = readStatusWord();

        // if (dbg_count < 50) {
        //     std::cout << "  StatusWord=0x" << std::hex << sw
        //               << " (expect mask 0x" << expect_mask << ")" << std::dec << std::endl;
        //     ++dbg_count;
        // }

        if ((sw & expect_mask) == expect_mask) return true;
        std::this_thread::sleep_for(milliseconds(1));
    }
    std::cerr << "Timeout waiting for state 0x" << std::hex << expect_mask << std::dec << "\n";
    return false;
}

bool MTDeviceController::waitForWorkingCounter(uint32_t timeout_ms) {
    auto t0 = steady_clock::now();
    while (duration_cast<milliseconds>(steady_clock::now() - t0).count() < timeout_ms) {
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        ecrt_domain_state(domain_, &domain_state_);
        ecrt_master_state(master_, &master_state_);
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);

        // WKC>0 认为域已开始交换
        if (domain_state_.working_counter > 0) return true;
        std::this_thread::sleep_for(milliseconds(2));
    }
    return false;
}

bool MTDeviceController::writeControlWord(uint16_t val) {
    if (!isMapped(offsets_.control_word)) {
        std::cerr << "Control word PDO not mapped\n";
        return false;
    }
    EC_WRITE_U16(domain_pd_ + offsets_.control_word, val);
    return true;
}

uint16_t MTDeviceController::readStatusWord() const {
    if (!isMapped(offsets_.status_word)) {
        // 不再打印噪声日志，以免刷屏
        return 0;
    }
    return EC_READ_U16(domain_pd_ + offsets_.status_word);
}

bool MTDeviceController::isMapped(uint32_t off) const {
    return off != kInvalidOffset;
}

bool MTDeviceController::verifyOffsetsMapped() const {
    const uint32_t* arr = reinterpret_cast<const uint32_t*>(&offsets_);
    // offsets_ 内部字段顺序与构造时设置保持一致
    constexpr size_t n =
        sizeof(MTDeviceOffsets) / sizeof(uint32_t);
    for (size_t i = 0; i < n; ++i) {
        if (arr[i] == kInvalidOffset) return false;
    }
    return true;
}

void MTDeviceController::setExternalRunningFlag(std::atomic<bool>* flag) {
    external_running_ = flag;
}
