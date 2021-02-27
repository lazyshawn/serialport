project(SerialPort)

cmake_minimum_required(VERSION 3.0)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

aux_source_directory(./src DIR_SRCS)

add_executable(comtest ${DIR_SRCS})

