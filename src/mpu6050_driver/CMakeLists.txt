cmake_minimum_required(VERSION 3.8)
project(mpu6050_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- 이 부분이 누락되면 include 폴더 안의 헤더를 찾지 못합니다 ---
include_directories(include)
# --------------------------------------------------------

add_executable(mpu6050_node src/mpu6050_driver.cpp)
ament_target_dependencies(mpu6050_node rclcpp sensor_msgs)
target_link_libraries(mpu6050_node i2c)

install(TARGETS mpu6050_node DESTINATION lib/${PROJECT_NAME})
ament_package()