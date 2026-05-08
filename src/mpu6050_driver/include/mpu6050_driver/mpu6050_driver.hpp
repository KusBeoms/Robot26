#ifndef MPU6050_DRIVER_HPP_
#define MPU6050_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>

class MPU6050Driver : public rclcpp::Node {
public:
    MPU6050Driver();

private:
    void init_mpu6050();
    void read_raw_data(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
    void madgwick_update_6dof(float ax, float ay, float az, float gx, float gy, float gz, float dt);
    
    // ### 추가됨: 타이머 콜백 함수 선언 ###
    void timer_callback(); 

    int fd_;
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; // 자이로 드리프트 보정값
    
    // 필터 파라미터
    const float beta = 0.1f;  // 추정 게인
    const float zeta = 0.01f; // 드리프트 수렴 게인

    rclcpp::Time last_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

#endif