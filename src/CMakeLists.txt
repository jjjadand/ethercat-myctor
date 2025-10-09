cmake_minimum_required(VERSION 3.10)
project(EtherCATMaster)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找EtherCAT库
find_library(ETHERCAT_LIB libethercat.so /usr/local/ethercat/lib)
find_path(ETHERCAT_INCLUDE_DIR ecrt.h /usr/local/ethercat/include)

if(NOT ETHERCAT_LIB OR NOT ETHERCAT_INCLUDE_DIR)
    message(FATAL_ERROR "EtherCAT library not found")
endif()

# 包含头文件
include_directories(${ETHERCAT_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 添加可执行文件
add_executable(ethercat_master
    main.cpp
    MTDeviceController.cpp
)

# 链接库
target_link_libraries(ethercat_master ${ETHERCAT_LIB} pthread rt)

# 设置编译器标志
target_compile_options(ethercat_master PRIVATE -O2 -Wall -Wextra)
