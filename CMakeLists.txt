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

set(FINISH_RECEIVER OFF)
set(TEST_RECEIVER OFF)
set(FINISH_PUBLISHER OFF)

add_compile_options(-Wall -Wpedantic -Wextra)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)

set(THIRD_DEPS
  Boost::system
  spdlog::spdlog
  fmt::fmt
)

set(ROS_DEPS
  rclcpp
)

include_directories(include)

add_executable(serial_sender src/serial_sender.cpp)
target_link_libraries(serial_sender PRIVATE ${THIRD_DEPS})

if(TEST_RECEIVER AND NOT FINISH_RECEIVER)
  message(WARNING "TEST_RECEIVER requires FINISH_RECEIVER. Disabling TEST_RECEIVER.")
  set(TEST_RECEIVER OFF)
endif()

if(FINISH_PUBLISHER AND NOT FINISH_RECEIVER)
  message(WARNING "FINISH_PUBLISHER requires FINISH_RECEIVER. Disabling FINISH_PUBLISHER.")
  set(FINISH_PUBLISHER OFF)
endif()

if(FINISH_RECEIVER)
  add_library(lib_serial_receiver STATIC src/serial_receiver.cpp)
  target_link_libraries(lib_serial_receiver PUBLIC ${THIRD_DEPS})
endif()

if(TEST_RECEIVER)
  add_executable(test_serial_receiver src/test_serial_receiver.cpp)
  target_link_libraries(test_serial_receiver PRIVATE lib_serial_receiver)
endif()

if(FINISH_PUBLISHER)
  add_executable(serial_relay_node src/msg_publisher.cpp)
  target_link_libraries(serial_relay_node PRIVATE lib_serial_receiver)
  ament_target_dependencies(serial_relay_node PUBLIC ${ROS_DEPS})
endif()

ament_package()
