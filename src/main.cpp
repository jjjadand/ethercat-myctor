// main.cpp
#include "MTDeviceController.h"
#include <csignal>
#include <atomic>

std::atomic<bool> running{true};

void signalHandler(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    running.store(false);
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    MTDeviceDriver driver;
    
    // 初始化驱动
    if (!driver.init("eth1", 0)) {
        std::cerr << "Failed to initialize driver" << std::endl;
        return -1;
    }
    
    // 设置控制回调
    driver.setControlCallback([](MTDeviceDriver::Status& status) {
        // 实时控制逻辑
        if (status.is_fault) {
            std::cerr << "Device fault detected!" << std::endl;
        }
    });
    
    // 设置位置模式
    if (!driver.setPositionMode()) {
        std::cerr << "Failed to set position mode" << std::endl;
        driver.shutdown();
        return -1;
    }
    
    // 使能设备
    if (!driver.enable()) {
        std::cerr << "Failed to enable device" << std::endl;
        driver.shutdown();
        return -1;
    }
    
    // 启动实时线程
    if (!driver.startRealtimeThread(1000)) {
        std::cerr << "Failed to start realtime thread" << std::endl;
        driver.shutdown();
        return -1;
    }
    
    // 主控制循环
    int32_t target_pos = 0;
    bool direction = true;
    
    while (running.load()) {
        // 生成运动指令
        if (direction) {
            target_pos += 1000;
            if (target_pos > 50000) direction = false;
        } else {
            target_pos -= 1000;
            if (target_pos < -50000) direction = true;
        }
        
        driver.setTargetPosition(target_pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 清理
    driver.stopRealtimeThread();
    driver.disable();
    driver.shutdown();
    
    return 0;
}
