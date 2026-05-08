# 빌드

cd ~/ros2_ws
# 전체 빌드
colcon build

# 또는 mpu6050_driver 패키지만 빌드
colcon build --packages-select mpu6050_driver

# 새 터미널 or 빌드 직후
cd ~/ros2_ws
source install/setup.bash

# 실행
ros2 run mpu6050_driver mpu6050_node