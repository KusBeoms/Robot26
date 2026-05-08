#include "mpu6050_driver/mpu6050_driver.hpp"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

// SMBus 통신을 위한 헤더
extern "C" {
    #include <i2c/smbus.h>
}

MPU6050Driver::MPU6050Driver() : Node("mpu6050_driver") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    
    // I2C 장치 열기
    fd_ = open("/dev/i2c-1", O_RDWR);
    if (fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "I2C 장치를 열 수 없습니다.");
        return;
    }
    
    if (ioctl(fd_, I2C_SLAVE, 0x68) < 0) {
        RCLCPP_ERROR(this->get_logger(), "MPU6050 주소(0x68) 접근 실패.");
        return;
    }
    
    init_mpu6050();
    
    last_time_ = this->get_clock()->now();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&MPU6050Driver::timer_callback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "MPU6050 Driver 가동 시작 (Madgwick Filter)");
}

void MPU6050Driver::init_mpu6050() {
    // MPU6050 깨우기 (Sleep 모드 해제)
    i2c_smbus_write_byte_data(fd_, 0x6B, 0x00);
}

void MPU6050Driver::read_raw_data(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    auto read_word = [this](int reg) {
        int val = i2c_smbus_read_word_data(fd_, reg);
        // 빅 엔디안 -> 리틀 엔디안 변환 및 부호 있는 정수 처리
        int16_t res = (int16_t)((val << 8) | (val >> 8));
        return res;
    };

    // 가속도 (±2g 기준 16384로 나눔)
    ax = read_word(0x3B) / 16384.0f;
    ay = read_word(0x3D) / 16384.0f;
    az = read_word(0x3F) / 16384.0f;

    // 자이로 (±250deg/s 기준 131로 나눔 -> rad/s 변환)
    gx = (read_word(0x43) / 131.0f) * (M_PI / 180.0f);
    gy = (read_word(0x45) / 131.0f) * (M_PI / 180.0f);
    gz = (read_word(0x47) / 131.0f) * (M_PI / 180.0f);
}

void MPU6050Driver::madgwick_update_6dof(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float s1, s2, s3, s4;
    float _2q1, _2q2, _2q3, _2q4, _4q1, _4q2, _4q3, _8q2, _8q3, q1q1, q2q2, q3q3, q4q4;

    // 가속도 정규화
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3; _2q4 = 2.0f * q4;
    _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _4q3 = 4.0f * q3;
    _8q2 = 8.0f * q2; _8q3 = 8.0f * q3;
    q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3; q4q4 = q4 * q4;

    s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
    s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
    s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
    
    norm = 1.0f / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    s1 *= norm; s2 *= norm; s3 *= norm; s4 *= norm;

    // 바이어스 보정
    float ex = _2q1 * s2 - _2q2 * s1 - _2q3 * s4 + _2q4 * s3;
    float ey = _2q1 * s3 + _2q2 * s4 - _2q3 * s1 - _2q4 * s2;
    float ez = _2q1 * s4 - _2q2 * s3 + _2q3 * s2 - _2q4 * s1;
    
    gyro_bias[0] += ex * dt * zeta;
    gyro_bias[1] += ey * dt * zeta;
    gyro_bias[2] += ez * dt * zeta;
    
    gx -= gyro_bias[0]; gy -= gyro_bias[1]; gz -= gyro_bias[2];

    float qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    float qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    float qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    float qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    q[0] += qDot1 * dt; q[1] += qDot2 * dt; q[2] += qDot3 * dt; q[3] += qDot4 * dt;
    norm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= norm; q[1] *= norm; q[2] *= norm; q[3] *= norm;
}

void MPU6050Driver::timer_callback() {
    float ax, ay, az, gx, gy, gz;
    read_raw_data(ax, ay, az, gx, gy, gz);

    auto current_time = this->get_clock()->now();
    float dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0) dt = 0.01f;

    madgwick_update_6dof(ax, ay, az, gx, gy, gz, dt);

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = current_time;
    msg.header.frame_id = "imu_link";
    
    msg.orientation.w = q[0];
    msg.orientation.x = q[1];
    msg.orientation.y = q[2];
    msg.orientation.z = q[3];

    publisher_->publish(msg);
}

// ### 링커 에러를 해결하는 핵심: main 함수 추가 ###
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPU6050Driver>());
    rclcpp::shutdown();
    return 0;
}