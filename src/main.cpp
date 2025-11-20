// main.cpp
#include "MTDeviceController.h"
#include <csignal>
#include <atomic>
#include <thread>
#include <iostream>
#include <ecrt.h>

std::atomic<bool> g_running{true};
bool g_direction = true;

void signalHandler(int sig) {
    std::cout << "Received signal " << sig << ", shutting down..." << std::endl;
    g_running.store(false);    
}


int main() {
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    MTDeviceController driver;
    driver.setExternalRunningFlag(&g_running);

    if (!driver.init("eno1", 0)) {
        std::cerr << "Failed to init\n";
        return -1;
    }

    if (!driver.startRealtimeThread(1000)) {
        std::cerr << "Failed to start thread\n";
        return -1;
    }

    // 主线程只等 Ctrl+C
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    driver.stopRealtimeThread();
    driver.shutdown();
    return 0;
}

// int main() {
//     signal(SIGINT,  signalHandler);
//     signal(SIGTERM, signalHandler);

//     MTDeviceController driver;

//     // 把外部 running 标志传进去，让实时线程也能看到
//     driver.setExternalRunningFlag(&g_running);

//     // 初始化驱动
//     if (!driver.init("eno1", 0)) {
//         std::cerr << "Failed to initialize driver" << std::endl;
//         return -1;
//     }

//         // 启动实时线程，频率可以先用 100Hz 比较合理
//     if (!driver.startRealtimeThread(100)) {
//         std::cerr << "Failed to start realtime thread" << std::endl;
//         driver.shutdown();
//         return -1;
//     }

//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//     if (!driver.setPositionMode()) {
//         std::cerr << "Failed to set position mode" << std::endl;
//         driver.shutdown();
//         return -1;
//     }


//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//     if (!driver.enable()) {
//         std::cerr << "Failed to enable device" << std::endl;
//         driver.shutdown();
//         return -1;
//     }




//     // 控制回调：这里只做“每周期发一个新目标”
//     driver.setControlCallback([&](MTDeviceController::Status &status) {
//         if (status.is_fault) {
//             std::cerr << "Device fault detected! status_word=0x"
//                       << std::hex << status.status_word << std::dec << std::endl;
//             g_running.store(false);
//             return;
//         }

//         if (!status.is_enabled) {
//             // 还没完全 OPERATION ENABLED，就先不发位置
//             //return;
//         }

//         // 注意：target_pos 必须是 static / 成员变量，不能每次都从 0 开始
//         static int32_t target_pos = 0;

//         if (g_direction) {
//             target_pos += 1000;
//             if (target_pos > 50000) g_direction = false;
//         } else {
//             target_pos -= 1000;
//             if (target_pos < -50000) g_direction = true;
//         }
//         target_pos = 20000;
//         driver.setTargetPosition(target_pos);
//     });



//     // 主线程只负责等退出信号
//     while (g_running.load()) {
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }

//     std::cout << "Main loop exiting, cleaning up..." << std::endl;

//     // 清理顺序：先停实时线程，再 disable，再 shutdown
//     driver.stopRealtimeThread();  
//     driver.disable();
//     driver.shutdown();

//     return 0;
// }
