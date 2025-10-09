// EtherCATMaster.h
#ifndef ETHERCAT_MASTER_H
#define ETHERCAT_MASTER_H

#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <unordered_map>
#include <cstring>
#include <ecrt.h>

// EtherCAT 从站状态
enum class SlaveState {
    INIT,
    PREOP,
    SAFEOP,
    OP,
    ERROR
};

// PDO 条目信息
struct PDOEntry {
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
    std::string name;
    ec_direction_t direction; // EC_DIR_INPUT or EC_DIR_OUTPUT
};

// 从站配置信息
struct SlaveConfig {
    int position;
    uint32_t vendor_id;
    uint32_t product_code;
    std::vector<PDOEntry> rx_pdos;  // 主站→从站
    std::vector<PDOEntry> tx_pdos;  // 从站→主站
    ec_sync_manager_t sync_managers[4];
    uint32_t sync_manager_count;
};

// 抽象从站接口类
class EtherCATSlave {
public:
    EtherCATSlave(int position, uint32_t vendor_id, uint32_t product_code);
    virtual ~EtherCATSlave();
    
    // 配置接口
    virtual bool configure(ec_domain_t* domain, ec_slave_config_t* config);
    virtual bool setupDomainMapping(ec_domain_t* domain);
    
    // 状态控制
    virtual bool setState(SlaveState state);
    virtual SlaveState getState() const;
    
    // PDO 数据访问
    virtual bool updateInputs();  // 读取输入PDO
    virtual bool updateOutputs(); // 写入输出PDO
    
    // 通用PDO访问方法
    template<typename T>
    bool writePDO(uint16_t index, uint8_t subindex, const T& value);
    
    template<typename T>
    bool readPDO(uint16_t index, uint8_t subindex, T& value);
    
    // 获取信息
    int getPosition() const { return position_; }
    uint32_t getVendorId() const { return vendor_id_; }
    uint32_t getProductCode() const { return product_code_; }
    ec_slave_config_t* getConfig() const { return config_; }
    
protected:
    virtual bool setupPDOMapping();
    
    int position_;
    uint32_t vendor_id_;
    uint32_t product_code_;
    ec_slave_config_t* config_{nullptr};
    ec_slave_config_state_t config_state_;
    
    // PDO 映射
    std::vector<PDOEntry> rx_pdos_;
    std::vector<PDOEntry> tx_pdos_;
    std::vector<ec_pdo_entry_reg_t> domain_registers_;
    
    // 域数据
    uint8_t* domain_data_{nullptr};
    size_t domain_offset_{0};
    
    mutable std::mutex data_mutex_;
};

// 通用 EtherCAT 主站类
class EtherCATMaster {
public:
    static EtherCATMaster& getInstance();
    
    // 禁止拷贝
    EtherCATMaster(const EtherCATMaster&) = delete;
    EtherCATMaster& operator=(const EtherCATMaster&) = delete;
    
    // 主站管理
    bool initialize(const std::string& network_interface = "eth0");
    void shutdown();
    
    // 从站管理
    bool addSlave(std::shared_ptr<EtherCATSlave> slave);
    bool removeSlave(int position);
    std::shared_ptr<EtherCATSlave> getSlave(int position);
    
    // 操作控制
    bool startCyclicOperation(uint32_t frequency_hz = 1000);
    bool stopCyclicOperation();
    bool activateMaster();
    
    // 状态查询
    bool isOperational() const { return is_operational_.load(); }
    bool isRunning() const { return is_running_.load(); }
    size_t getSlaveCount() const { 
        std::lock_guard<std::mutex> lock(master_mutex_);
        return slaves_.size();
    }
    uint32_t getUpdateCounter() const { return update_counter_.load(); }
    
    // 回调设置
    void setPreCycleCallback(std::function<void()> callback);
    void setPostCycleCallback(std::function<void()> callback);
    void setErrorCallback(std::function<void(const std::string&)> callback);
    
    // 诊断信息
    struct MasterStatus {
        ec_master_state_t master_state;
        ec_domain_state_t domain_state;
        uint32_t slave_count;
        uint32_t update_counter;
        bool is_operational;
        bool is_running;
    };
    
    MasterStatus getStatus() const;

private:
    EtherCATMaster();
    ~EtherCATMaster();
    
    void cyclicTaskThread();
    void updateMasterState();
    bool checkDomainState();
    void handleError(const std::string& error_message);
    
    static EtherCATMaster* instance_;
    
    // EtherCAT 对象
    ec_master_t* master_{nullptr};
    ec_domain_t* domain_{nullptr};
    
    // 状态信息
    ec_master_state_t master_state_;
    ec_domain_state_t domain_state_;
    
    // 从站管理
    std::unordered_map<int, std::shared_ptr<EtherCATSlave>> slaves_;
    std::vector<ec_slave_config_t*> slave_configs_;
    std::vector<ec_pdo_entry_reg_t> domain_registry_;
    
    // 线程控制
    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_operational_{false};
    std::thread cyclic_thread_;
    mutable std::mutex master_mutex_;
    
    // 回调函数
    std::function<void()> pre_cycle_callback_;
    std::function<void()> post_cycle_callback_;
    std::function<void(const std::string&)> error_callback_;
    
    // 运行参数
    std::atomic<uint32_t> update_frequency_hz_{1000};
    std::atomic<uint32_t> update_counter_{0};
    std::string interface_;
    
    // 性能监控
    std::atomic<uint32_t> lost_frames_{0};
    std::atomic<uint32_t> total_cycles_{0};
};

#endif // ETHERCAT_MASTER_H