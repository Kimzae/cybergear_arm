cmake_minimum_required(VERSION 3.16)
project(cybergear_arm LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # 제어 루프는 최적화 빌드로!
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)   # sudo apt install libyaml-cpp-dev

# ---- 1) ROS 와 무관한 코어 라이브러리 (프로토콜/버스/모델/제어기) ----
add_library(cybergear_core
  src/cybergear_protocol.cpp
  src/serial_at_transport.cpp
  src/socketcan_transport.cpp
  src/sim_transport.cpp
  src/cybergear_bus.cpp
  src/arm_dynamics.cpp
  src/dh_dynamics.cpp
  src/smc_controller.cpp
)
target_include_directories(cybergear_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cybergear_core PUBLIC Eigen3::Eigen yaml-cpp::yaml-cpp)

# ---- 2) ROS2 노드 ----
add_executable(arm_controller src/arm_controller_node.cpp)
target_link_libraries(arm_controller cybergear_core pthread)
ament_target_dependencies(arm_controller rclcpp sensor_msgs std_msgs trajectory_msgs)

# ---- 3) 하드웨어 없이 돌리는 테스트 프로그램 ----
add_executable(protocol_selftest test/protocol_selftest.cpp)
target_link_libraries(protocol_selftest cybergear_core)
add_executable(smc_sim_test test/smc_sim_test.cpp)
target_link_libraries(smc_sim_test cybergear_core)
add_executable(model_test test/model_test.cpp)
target_link_libraries(model_test cybergear_core)
add_executable(chattering_sim_test test/chattering_sim_test.cpp)
target_link_libraries(chattering_sim_test cybergear_core)

install(TARGETS cybergear_core arm_controller protocol_selftest smc_sim_test model_test chattering_sim_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sine_reference.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
