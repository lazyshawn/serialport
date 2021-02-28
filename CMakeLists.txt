# 项目名称
project(SerialPort)

# CMake版本要求
cmake_minimum_required(VERSION 3.0)

# 头文件路径
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# 源文件路径
aux_source_directory(./src DIR_SRCS)

# 生成动态链接库
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
add_library(serialport SHARED ./src/serialport.cpp ./include/serialport.h)

# 生成可执行文件
add_executable(comtest ${DIR_SRCS})

