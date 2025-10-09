// MTDeviceDriver.cpp
#include "MTDeviceController.h"
#include <cstring>

using namespace std::chrono;

// 对象字典索引定义
namespace ObjectDictionary {
    constexpr uint16_t CONTROL_WORD = 0x6040;
    constexpr uint16_t STATUS_WORD = 0x6041;
    constexpr uint16_t OPERATION_MODE = 0x6060;
    constexpr uint16_t OPERATION_MODE_DISPLAY = 0x6061;
    constexpr uint16_t ACTUAL_POSITION = 0x6064;
    constexpr uint16_t ACTUAL_VELOCITY = 0x606C;
    constexpr uint16_t ACTUAL_TORQUE = 0x6077;
    constexpr uint16_t TARGET_POSITION = 0x607A;
    constexpr uint16_t TARGET_VELOCITY = 0x60FF;
    constexpr uint16_t TARGET_TORQUE = 0x6071;
    constexpr uint16_t MAX_TORQUE = 0x6072;
    constexpr uint16_t ERROR_CODE = 0x603F;
}

// 控制字命令
namespace ControlWord {
    constexpr uint16_t SHUTDOWN = 0x0006;
    constexpr uint16_t SWITCH_ON = 0x0007;
    constexpr uint16_t ENABLE_OPERATION = 0x000F;
    constexpr uint16_t QUICK_STOP = 0x0002;
    constexpr uint16_t FAULT_RESET = 0x0080;
}

// 状态字位
namespace StatusWord {
    constexpr uint16_t READY_TO_SWITCH_ON = 0x0001;
    constexpr uint16_t SWITCHED_ON = 0x0002;
    constexpr uint16_t OPERATION_ENABLED = 0x0004;
    constexpr uint16_t FAULT = 0x0008;
    constexpr uint16_t TARGET_REACHED = 0x0400;
}

MTDeviceDriver::MTDeviceDriver() 
    : master_(nullptr), domain_(nullptr), slave_config_(nullptr), domain_pd_(nullptr) {
    memset(&master_state_, 0, sizeof(master_state_));
    memset(&domain_state_, 0, sizeof(domain_state_));
    memset(&offsets_, 0, sizeof(offsets_));
}

MTDeviceDriver::~MTDeviceDriver() {
    shutdown();
}

bool MTDeviceDriver::init(const char* interface, int slave_position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << "Initializing MT Device Driver on interface: " << interface << std::endl;
    
    // 1. 请求主站
    master_ = ecrt_request_master(0);
    if (!master_) {
        std::cerr << "Failed to request EtherCAT master" << std::endl;
        return false;
    }
    
    // 2. 创建域
    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        std::cerr << "Failed to create domain" << std::endl;
        ecrt_release_master(master_);
        master_ = nullptr;
        return false;
    }
    
    // 3. 配置从站 (MT Device)
    slave_config_ = ecrt_master_slave_config(
        master_, 0, slave_position, 0x00202008, 0x00000000); // 使用脉塔的 Vendor ID 和 Product Code
    
    if (!slave_config_) {
        std::cerr << "Failed to get slave configuration" << std::endl;
        ecrt_release_master(master_);
        master_ = nullptr;
        return false;
    }
    
    // 4. 配置 PDO
    ec_pdo_entry_reg_t domain_regs[] = {
        // RxPDO (主站 -> 从站)
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::CONTROL_WORD, 0, &offsets_.control_word},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::OPERATION_MODE, 0, &offsets_.operation_mode},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::TARGET_POSITION, 0, &offsets_.target_position},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::TARGET_VELOCITY, 0, &offsets_.target_velocity},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::TARGET_TORQUE, 0, &offsets_.target_torque},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::MAX_TORQUE, 0, &offsets_.max_torque},
        
        // TxPDO (从站 -> 主站)
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::STATUS_WORD, 0, &offsets_.status_word},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::OPERATION_MODE_DISPLAY, 0, &offsets_.operation_mode_display},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::ACTUAL_POSITION, 0, &offsets_.actual_position},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::ACTUAL_VELOCITY, 0, &offsets_.actual_velocity},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::ACTUAL_TORQUE, 0, &offsets_.actual_torque},
        {0, slave_position, 0x00202008, 0x00000000, ObjectDictionary::ERROR_CODE, 0, &offsets_.error_code},
        {}
    };
    
    if (ecrt_domain_reg_pdo_entry_list(domain_, domain_regs)) {
        std::cerr << "PDO entry registration failed!" << std::endl;
        return false;
    }
    
    // 5. 激活主站
    if (ecrt_master_activate(master_)) {
        std::cerr << "Master activation failed!" << std::endl;
        return false;
    }
    
    // 6. 获取域数据指针
    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_) {
        std::cerr << "Failed to get domain data pointer" << std::endl;
        return false;
    }
    
    std::cout << "MT Device Driver initialized successfully" << std::endl;
    return true;
}

void MTDeviceDriver::shutdown() {
    stopRealtimeThread();
    disable();
    
    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
    
    std::cout << "MT Device Driver shutdown" << std::endl;
}

bool MTDeviceDriver::enable() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << "Enabling MT Device..." << std::endl;
    
    // 状态机转换: 6 -> 7 -> 15
    if (!writeControlWord(ControlWord::SHUTDOWN)) return false;
    if (!waitForState(StatusWord::READY_TO_SWITCH_ON)) return false;
    
    if (!writeControlWord(ControlWord::SWITCH_ON)) return false;
    if (!waitForState(StatusWord::SWITCHED_ON)) return false;
    
    if (!writeControlWord(ControlWord::ENABLE_OPERATION)) return false;
    if (!waitForState(StatusWord::OPERATION_ENABLED)) return false;
    
    std::cout << "MT Device enabled successfully" << std::endl;
    return true;
}

bool MTDeviceDriver::disable() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << "Disabling MT Device..." << std::endl;
    
    if (!writeControlWord(0x0000)) return false;
    if (!waitForState(StatusWord::READY_TO_SWITCH_ON)) return false;
    
    std::cout << "MT Device disabled successfully" << std::endl;
    return true;
}

bool MTDeviceDriver::setPositionMode() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (offsets_.operation_mode == 0) {
        std::cerr << "Operation mode PDO not mapped" << std::endl;
        return false;
    }
    
    EC_WRITE_S8(domain_pd_ + offsets_.operation_mode, 0x08); // CSP模式
    return true;
}

bool MTDeviceDriver::setTargetPosition(int32_t position) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (offsets_.target_position == 0) {
        std::cerr << "Target position PDO not mapped" << std::endl;
        //return false;
    }
    
    EC_WRITE_S32(domain_pd_ + offsets_.target_position, position);
    return true;
}

MTDeviceDriver::Status MTDeviceDriver::getStatus() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Status status;
    
    // 读取所有状态数据
    if (offsets_.status_word > 0)
        status.status_word = EC_READ_U16(domain_pd_ + offsets_.status_word);
    if (offsets_.actual_position > 0)
        status.actual_position = EC_READ_S32(domain_pd_ + offsets_.actual_position);
    if (offsets_.actual_velocity > 0)
        status.actual_velocity = EC_READ_S32(domain_pd_ + offsets_.actual_velocity);
    if (offsets_.actual_torque > 0)
        status.actual_torque = EC_READ_S16(domain_pd_ + offsets_.actual_torque);
    if (offsets_.error_code > 0)
        status.error_code = EC_READ_U16(domain_pd_ + offsets_.error_code);
    if (offsets_.operation_mode_display > 0)
        status.operation_mode_display = EC_READ_S8(domain_pd_ + offsets_.operation_mode_display);
    
    // 解析状态
    status.is_enabled = (status.status_word & StatusWord::OPERATION_ENABLED) != 0;
    status.is_fault = (status.status_word & StatusWord::FAULT) != 0;
    status.target_reached = (status.status_word & StatusWord::TARGET_REACHED) != 0;
    
    return status;
}

bool MTDeviceDriver::startRealtimeThread(uint32_t frequency_hz) {
    if (running_.load()) {
        std::cout << "Realtime thread already running" << std::endl;
        return true;
    }
    
    running_.store(true);
    realtime_thread_ = std::thread(&MTDeviceDriver::realtimeThreadFunc, this, frequency_hz);
    
    std::cout << "Started realtime thread at " << frequency_hz << " Hz" << std::endl;
    return true;
}

void MTDeviceDriver::stopRealtimeThread() {
    if (!running_.load()) return;
    
    running_.store(false);
    if (realtime_thread_.joinable()) {
        realtime_thread_.join();
    }
    
    std::cout << "Stopped realtime thread" << std::endl;
}

void MTDeviceDriver::realtimeThreadFunc(uint32_t frequency_hz) {
    auto cycle_time = microseconds(1000000 / frequency_hz);
    auto next_wake_time = steady_clock::now();
    
    std::cout << "Realtime thread started" << std::endl;
    
    while (running_.load()) {
        auto cycle_start = steady_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(mutex_);
            
            // EtherCAT通信周期
            ecrt_master_receive(master_);
            ecrt_domain_process(domain_);
            
            // 用户控制回调
            if (control_callback_) {
                Status status = getStatus();
                control_callback_(status);
            }
            
            ecrt_domain_queue(domain_);
            ecrt_master_send(master_);
        }
        
        // 精确周期控制
        next_wake_time += cycle_time;
        std::this_thread::sleep_until(next_wake_time);
    }
    
    std::cout << "Realtime thread stopped" << std::endl;
}


void MTDeviceDriver::setControlCallback(std::function<void(Status&)> callback) {
    control_callback_ = std::move(callback);
}


// 辅助函数实现
bool MTDeviceDriver::waitForState(uint16_t expected_state, uint32_t timeout_ms) {
    auto start_time = steady_clock::now();
    
    while (duration_cast<milliseconds>(steady_clock::now() - start_time).count() < timeout_ms) {
        // 执行一个EtherCAT周期来更新数据
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);
        
        uint16_t status_word = readStatusWord();
        if ((status_word & expected_state) == expected_state) {
            return true;
        }
        
        std::this_thread::sleep_for(milliseconds(1));
    }
    
    std::cerr << "Timeout waiting for state 0x" << std::hex << expected_state << std::dec << std::endl;
    return false;
}

bool MTDeviceDriver::writeControlWord(uint16_t value) {
    if (offsets_.control_word == 0) {
        std::cerr << "Control word PDO not mapped" << std::endl;
        return false;
    }
    
    EC_WRITE_U16(domain_pd_ + offsets_.control_word, value);
    return true;
}

uint16_t MTDeviceDriver::readStatusWord() {
    if (offsets_.status_word == 0) {
        std::cerr << "Status word PDO not mapped" << std::endl;
        return 0;
    }
    
    return EC_READ_U16(domain_pd_ + offsets_.status_word);
}
