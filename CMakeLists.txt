cmake_minimum_required(VERSION 3.22)
project(serial_comm)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_DIR build)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# 设置为ON，编译接收器和测试程序
set(FINISH_RECEIVER ON)
set(TEST_RECEIVER ON)
set(FINISH_PUBLISHER OFF)

add_compile_options(-Wall -Wpedantic -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)

# 使用pkg-config查找libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

set(THIRD_DEPS
  Boost::system
  spdlog::spdlog
  fmt::fmt
)

set(ROS_DEPS
  rclcpp
)

# 添加libserial的包含目录
include_directories(include ${LIBSERIAL_INCLUDE_DIRS})

add_executable(serial_sender src/serial_sender.cpp)
target_link_libraries(serial_sender PRIVATE ${THIRD_DEPS} ${LIBSERIAL_LIBRARIES})

# serial_receiver库
add_library(lib_serial_receiver STATIC src/serial_receiver.cpp)
target_link_libraries(lib_serial_receiver PUBLIC ${THIRD_DEPS} ${LIBSERIAL_LIBRARIES})
target_include_directories(lib_serial_receiver PUBLIC ${LIBSERIAL_INCLUDE_DIRS})

# 测试程序
add_executable(test_serial_receiver src/test_serial_receiver.cpp)
target_link_libraries(test_serial_receiver PRIVATE lib_serial_receiver ${THIRD_DEPS} ${LIBSERIAL_LIBRARIES})
target_include_directories(test_serial_receiver PRIVATE ${LIBSERIAL_INCLUDE_DIRS})

ament_package()
