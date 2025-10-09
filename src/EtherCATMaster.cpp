// EtherCATMaster.cpp
#include "EtherCATMaster.h"
#include <chrono>
#include <sstream>
#include <iomanip>

using namespace std::chrono;

// 静态成员初始化
EtherCATMaster* EtherCATMaster::instance_ = nullptr;

EtherCATMaster& EtherCATMaster::getInstance() {
    if (!instance_) {
        instance_ = new EtherCATMaster();
    }
    return *instance_;
}

EtherCATMaster::EtherCATMaster() {
    memset(&master_state_, 0, sizeof(master_state_));
    memset(&domain_state_, 0, sizeof(domain_state_));
}

EtherCATMaster::~EtherCATMaster() {
    shutdown();
}

bool EtherCATMaster::initialize(const std::string& network_interface) {
    std::lock_guard<std::mutex> lock(master_mutex_);
    
    if (is_initialized_.load()) {
        std::cout << "EtherCAT Master already initialized" << std::endl;
        return true;
    }
    
    interface_ = network_interface;
    
    std::cout << "Initializing EtherCAT Master on interface: " << interface_ << std::endl;
    
    // 获取EtherCAT主站实例
    master_ = ecrt_request_master(0);
    if (!master_) {
        handleError("Failed to request EtherCAT master");
        return false;
    }
    
    // 创建过程数据域
    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        handleError("Failed to create domain");
        ecrt_release_master(master_);
        master_ = nullptr;
        return false;
    }
    
    // 配置网络接口
    if (ecrt_master_set_send_interval(master_, 1000) != 0) { // 1ms默认发送间隔
        std::cerr << "Warning: Failed to set send interval" << std::endl;
    }
    
    std::cout << "EtherCAT Master initialized successfully" << std::endl;
    is_initialized_.store(true);
    return true;
}

void EtherCATMaster::shutdown() {
    if (!is_initialized_.load()) return;
    
    std::cout << "Shutting down EtherCAT Master..." << std::endl;
    
    stopCyclicOperation();
    is_running_.store(false);
    
    if (cyclic_thread_.joinable()) {
        cyclic_thread_.join();
    }
    
    std::lock_guard<std::mutex> lock(master_mutex_);
    
    // 释放从站配置
    for (auto& config : slave_configs_) {
        if (config) {
            ecrt_slave_config_release(config);
        }
    }
    slave_configs_.clear();
    slaves_.clear();
    
    // 释放域和主站
    if (domain_) {
        ecrt_master_release_domain(domain_);
        domain_ = nullptr;
    }
    
    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
    
    is_initialized_.store(false);
    is_operational_.store(false);
    
    std::cout << "EtherCAT Master shutdown complete" << std::endl;
}

bool EtherCATMaster::addSlave(std::shared_ptr<EtherCATSlave> slave) {
    std::lock_guard<std::mutex> lock(master_mutex_);
    
    if (!is_initialized_.load()) {
        handleError("EtherCAT Master not initialized");
        return false;
    }
    
    int position = slave->getPosition();
    if (slaves_.find(position) != slaves_.end()) {
        handleError("Slave at position " + std::to_string(position) + " already exists");
        return false;
    }
    
    // 创建从站配置
    ec_slave_config_t* config = ecrt_master_slave_config(
        master_, 0, position, 
        slave->getVendorId(), slave->getProductCode());
    
    if (!config) {
        handleError("Failed to create slave config for position " + std::to_string(position));
        return false;
    }
    
    slave_configs_.push_back(config);
    
    // 配置从站
    if (!slave->configure(domain_, config)) {
        handleError("Failed to configure slave at position " + std::to_string(position));
        ecrt_slave_config_release(config);
        return false;
    }
    
    slaves_[position] = slave;
    
    std::cout << "Added slave at position " << position 
              << " (Vendor: 0x" << std::hex << slave->getVendorId() 
              << ", Product: 0x" << slave->getProductCode() << ")" << std::dec << std::endl;
    
    return true;
}

bool EtherCATMaster::startCyclicOperation(uint32_t frequency_hz) {
    if (!is_initialized_.load()) {
        handleError("EtherCAT Master not initialized");
        return false;
    }
    
    if (is_running_.load()) {
        std::cout << "Cyclic operation already running" << std::endl;
        return true;
    }
    
    // 激活主站
    if (!activateMaster()) {
        handleError("Failed to activate master");
        return false;
    }
    
    update_frequency_hz_.store(frequency_hz);
    is_running_.store(true);
    cyclic_thread_ = std::thread(&EtherCATMaster::cyclicTaskThread, this);
    
    std::cout << "Started cyclic operation at " << frequency_hz << " Hz" << std::endl;
    return true;
}

bool EtherCATMaster::activateMaster() {
    std::lock_guard<std::mutex> lock(master_mutex_);
    
    // 注册域
    if (domain_registry_.empty()) {
        handleError("No PDO entries registered for domain");
        return false;
    }
    
    // 注册PDO条目
    if (ecrt_domain_reg_pdo_entry_list(domain_, domain_registry_.data()) != 0) {
        handleError("Failed to register PDO entries");
        return false;
    }
    
    // 激活主站
    if (ecrt_master_activate(master_) != 0) {
        handleError("Failed to activate master");
        return false;
    }
    
    // 获取域数据指针
    uint8_t* domain_data = ecrt_domain_data(domain_);
    if (!domain_data) {
        handleError("Failed to get domain data pointer");
        return false;
    }
    
    // 设置从站的域数据指针
    for (auto& [position, slave] : slaves_) {
        // 这里需要根据实际的PDO映射设置数据指针
        // 简化实现，实际应用中需要更详细的配置
    }
    
    is_operational_.store(true);
    std::cout << "EtherCAT Master activated successfully" << std::endl;
    
    return true;
}

void EtherCATMaster::cyclicTaskThread() {
    auto cycle_time = microseconds(1000000 / update_frequency_hz_.load());
    auto next_wake_time = steady_clock::now();
    auto last_state_check = steady_clock::now();
    
    std::cout << "Cyclic task thread started" << std::endl;
    
    while (is_running_.load()) {
        auto cycle_start = steady_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(master_mutex_);
            
            // 前置回调
            if (pre_cycle_callback_) {
                pre_cycle_callback_();
            }
            
            // EtherCAT通信周期
            ecrt_master_receive(master_);
            ecrt_domain_process(domain_);
            
            // 更新所有从站的输入数据
            for (auto& [position, slave] : slaves_) {
                slave->updateInputs();
            }
            
            // 后置回调（用户控制逻辑）
            if (post_cycle_callback_) {
                post_cycle_callback_();
            }
            
            // 更新所有从站的输出数据
            for (auto& [position, slave] : slaves_) {
                slave->updateOutputs();
            }
            
            ecrt_domain_queue(domain_);
            ecrt_master_send(master_);
        }
        
        update_counter_++;
        total_cycles_++;
        
        // 定期检查主站状态
        auto now = steady_clock::now();
        if (duration_cast<milliseconds>(now - last_state_check).count() >= 1000) {
            updateMasterState();
            last_state_check = now;
        }
        
        // 精确周期控制
        next_wake_time += cycle_time;
        
        // 检查是否落后于计划
        auto wake_delay = duration_cast<microseconds>(steady_clock::now() - next_wake_time);
        if (wake_delay.count() > 1000) { // 超过1ms延迟
            lost_frames_++;
            std::cerr << "Cycle delayed by " << wake_delay.count() << "us" << std::endl;
            next_wake_time = steady_clock::now() + cycle_time;
        }
        
        std::this_thread::sleep_until(next_wake_time);
    }
    
    std::cout << "Cyclic task thread stopped" << std::endl;
}

void EtherCATMaster::updateMasterState() {
    ecrt_master_state(master_, &master_state_);
    ecrt_domain_state(domain_, &domain_state_);
    
    // 检查错误
    if (master_state_.slaves_responding != slaves_.size()) {
        std::cerr << "Warning: Only " << master_state_.slaves_responding 
                  << " of " << slaves_.size() << " slaves responding" << std::endl;
    }
    
    if (domain_state_.working_counter != domain_state_.expected_working_counter) {
        std::cerr << "Warning: Domain working counter mismatch: " 
                  << domain_state_.working_counter << "/" 
                  << domain_state_.expected_working_counter << std::endl;
    }
    
    if (master_state_.link_up == 0) {
        handleError("EtherCAT link down");
    }
}

EtherCATMaster::MasterStatus EtherCATMaster::getStatus() const {
    MasterStatus status;
    status.master_state = master_state_;
    status.domain_state = domain_state_;
    status.slave_count = slaves_.size();
    status.update_counter = update_counter_.load();
    status.is_operational = is_operational_.load();
    status.is_running = is_running_.load();
    return status;
}

void EtherCATMaster::handleError(const std::string& error_message) {
    std::cerr << "EtherCAT Error: " << error_message << std::endl;
    if (error_callback_) {
        error_callback_(error_message);
    }
}

// EtherCATSlave 实现
EtherCATSlave::EtherCATSlave(int position, uint32_t vendor_id, uint32_t product_code)
    : position_(position), vendor_id_(vendor_id), product_code_(product_code) {
}

EtherCATSlave::~EtherCATSlave() {
}

bool EtherCATSlave::configure(ec_domain_t* domain, ec_slave_config_t* config) {
    config_ = config;
    return setupPDOMapping() && setupDomainMapping(domain);
}

bool EtherCATSlave::setupPDOMapping() {
    // 子类需要重写此方法来实现具体的PDO映射
    // 这里提供通用框架
    return true;
}

bool EtherCATSlave::setupDomainMapping(ec_domain_t* domain) {
    // 子类需要重写此方法来实现域映射
    // 这里提供通用框架
    return true;
}

template<typename T>
bool EtherCATSlave::writePDO(uint16_t index, uint8_t subindex, const T& value) {
    // 子类需要实现具体的PDO写入逻辑
    return false;
}

template<typename T>
bool EtherCATSlave::readPDO(uint16_t index, uint8_t subindex, T& value) {
    // 子类需要实现具体的PDO读取逻辑
    return false;
}

bool EtherCATSlave::updateInputs() {
    // 子类需要实现输入数据更新
    return true;
}

bool EtherCATSlave::updateOutputs() {
    // 子类需要实现输出数据更新
    return true;
}