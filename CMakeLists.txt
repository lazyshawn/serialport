############################################################################
# ==>> Basic Setting
############################################################################
cmake_minimum_required(VERSION 3.5)
# Project name | ${PROJECT_NAME}
project(demo)

############################################################################
# ==>> Generate a library
############################################################################
# Output of the library
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
# Generate the library from the library sources
add_library(demo_lib SHARED
  lib/src/serialport.cpp
)

target_include_directories(demo_lib PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

############################################################################
# ==>> Create an executable
############################################################################
# Source directories
aux_source_directory(${CMAKE_SOURCE_DIR}/src DIR_SRCS)
# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
# Add an executable with the above sources
add_executable(demo_bin
  ${DIR_SRCS}
)

# Link the demo_lib target with the demo_bin target
target_link_libraries(demo_bin PRIVATE
  demo_lib
)

