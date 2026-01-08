cmake_minimum_required(VERSION 3.8)
project(virtual_motor_device)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # 添加这一行！
  ${CMAKE_CURRENT_SOURCE_DIR}/src      # 可选，如果还有其他头文件在src中
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 尝试查找CANopen库，但不强制要求
find_package(canopen_base_driver QUIET)
find_package(canopen_core QUIET)

# 创建库
add_library(${PROJECT_NAME}
  src/virtual_motor_device.cpp
)

# 基础依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

# 如果有CANopen库，添加额外依赖
if(canopen_base_driver_FOUND AND canopen_core_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_CANOPEN_LIBRARIES)
  ament_target_dependencies(${PROJECT_NAME}
    canopen_base_driver
    canopen_core
  )
  message(STATUS "找到CANopen库，启用完整功能")
else()
  message(STATUS "未找到CANopen库，使用简化版本")
endif()

# 创建可执行文件
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()