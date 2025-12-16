/* include //{ */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

//}

/* ICM-42688-P registers //{ */

#define PWR_MGMT0 0x4E
#define INTF_CONFIG5 0x7B
#define GYRO_CONFIG0 0x4F
#define GYRO_CONFIG1 0x51
#define ACCEL_CONFIG0 0x50
#define ACCEL_CONFIG1 0x53
#define GYRO_ACCEL_CONFIG0 0x52
#define FIFO_CONFIG 0x16
#define FIFO_CONFIG1 0x5F
#define FIFO_DATA 0x30
#define SIGNAL_PATH_RESET 0x4B

#define INT_CONFIG 0x14

#define INT_CONFIG0 0x63
#define INT_CONFIG1 0x64
#define INT_SOURCE0 0x65

#define PWR_MGMT0_GYRO_ON_LN 0x0C
#define PWR_MGMT0_ACC_ON_LN 0x03

#define INTF_CONFIG5_PIN9_CLKIN 0x04

#define GYRO_UI_FILT_ORD_1 0x00  // 00
#define GYRO_UI_FILT_ORD_2 0x04  // 01
#define GYRO_UI_FILT_ORD_3 0x08  // 10

#define ACCEL_UI_FILT_ORD_1 0x00  // 00
#define ACCEL_UI_FILT_ORD_2 0x08  // 01
#define ACCEL_UI_FILT_ORD_3 0x10  // 10

#define ACCEL_UI_FILT_BW_6 0x60
#define ACCEL_UI_FILT_BW_7 0x70

#define GYRO_UI_FILT_BW_6 0x06
#define GYRO_UI_FILT_BW_7 0x07

#define INT_CONFIG_INT1_ACTIVE_HIGH_PUSH_PULL 0x03

#define GYRO_CONFIG0_GYRO_ODR_32_KHZ 0x01
#define GYRO_CONFIG0_GYRO_ODR_16_KHZ 0x02
#define GYRO_CONFIG0_GYRO_ODR_8_KHZ 0x03
#define GYRO_CONFIG0_GYRO_ODR_4_KHZ 0x04
#define GYRO_CONFIG0_GYRO_ODR_2_KHZ 0x05
#define GYRO_CONFIG0_GYRO_ODR_1_KHZ 0x06
#define GYRO_CONFIG0_GYRO_ODR_200_HZ 0x07
#define GYRO_CONFIG0_GYRO_ODR_100_HZ 0x08
#define GYRO_CONFIG0_GYRO_ODR_50_HZ 0x09
#define GYRO_CONFIG0_GYRO_ODR_25_HZ 0x0A
#define GYRO_CONFIG0_GYRO_ODR_12_HZ 0x0B
#define GYRO_CONFIG0_GYRO_ODR_500_HZ 0x0F

#define GYRO_CONFIG0_GYRO_FS_SEL_2000_DPS 0x00
#define GYRO_CONFIG0_GYRO_FS_SEL_1000_DPS 0x20
#define GYRO_CONFIG0_GYRO_FS_SEL_500_DPS 0x40
#define GYRO_CONFIG0_GYRO_FS_SEL_250_DPS 0x60
#define GYRO_CONFIG0_GYRO_FS_SEL_125_DPS 0x80
#define GYRO_CONFIG0_GYRO_FS_SEL_62_DPS 0xA0
#define GYRO_CONFIG0_GYRO_FS_SEL_31_DPS 0xC0
#define GYRO_CONFIG0_GYRO_FS_SEL_15_DPS 0xE0

#define ACCEL_CONFIG0_ACCEL_ODR_32_KHZ 0x01
#define ACCEL_CONFIG0_ACCEL_ODR_16_KHZ 0x02
#define ACCEL_CONFIG0_ACCEL_ODR_8_KHZ 0x03
#define ACCEL_CONFIG0_ACCEL_ODR_4_KHZ 0x04
#define ACCEL_CONFIG0_ACCEL_ODR_2_KHZ 0x05
#define ACCEL_CONFIG0_ACCEL_ODR_1_KHZ 0x06
#define ACCEL_CONFIG0_ACCEL_ODR_200_HZ 0x07
#define ACCEL_CONFIG0_ACCEL_ODR_100_HZ 0x08
#define ACCEL_CONFIG0_ACCEL_ODR_50_HZ 0x09
#define ACCEL_CONFIG0_ACCEL_ODR_25_HZ 0x0A
#define ACCEL_CONFIG0_ACCEL_ODR_12_HZ 0x0B
#define ACCEL_CONFIG0_ACCEL_ODR_6_HZ 0x0C
#define ACCEL_CONFIG0_ACCEL_ODR_3_HZ 0x0D
#define ACCEL_CONFIG0_ACCEL_ODR_1_HZ 0x0E
#define ACCEL_CONFIG0_ACCEL_ODR_500_HZ 0x0F

#define ACCEL_CONFIG0_ACCEL_FS_SEL_16_G 0x00
#define ACCEL_CONFIG0_ACCEL_FS_SEL_8_G 0x20
#define ACCEL_CONFIG0_ACCEL_FS_SEL_4_G 0x40
#define ACCEL_CONFIG0_ACCEL_FS_SEL_2_G 0x60

#define FIFO_CONFIG_FIFO_MODE_STREAM 0x40

#define FIFO_CONFIG1_FIFO_HIRES_EN 0x10
#define FIFO_CONFIG1_FIFO_GYRO_EN 0x02
#define FIFO_CONFIG1_FIFO_ACCEL_EN 0x01
#define FIFO_CONFIG1_FIFO_RESUME_PARTIAL_RD 0x40

#define SIGNAL_PATH_RESET_FIFO_FLUSH 0x02

#define INT_CONFIG0_INT 0x08

#define INT_CONFIG1_INT 0x00

#define INT_SOURCE0_FIFO_THS_INT1_EN 0x08

//}

#define INTERRUPT_PIN 11  // pin 11 equals GPIO 17
const float G           = 9.80665;
const float DEG2RAD     = 57.2958;
const float ACC_SCALER  = 8192;
const float GYRO_SCALER = 16.384;


namespace mrs_icm_imu_driver {

/* class MrsIcmImuDriver //{ */

class MrsIcmImuDriver : public rclcpp::Node {
public:
  MrsIcmImuDriver(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  void initialize();

  rclcpp::TimerBase::SharedPtr timer_preinitialization_;
  void                         timerPreInitialization();
  std::atomic<bool>            is_initialized_ = false;

  rclcpp::TimerBase::SharedPtr timer_imu_;
  void       timerImu();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

//}

/* MrsIcmImuDriver() //{ */

MrsIcmImuDriver::MrsIcmImuDriver(rclcpp::NodeOptions options) : Node("IcmImuDriver", options) {
  timer_preinitialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&MrsIcmImuDriver::timerPreInitialization, this));
}

//}

/* timerPreInitialization() //{ */

void MrsIcmImuDriver::timerPreInitialization() {
  node_  = this->shared_from_this();
  clock_ = node_->get_clock();

  initialize();
  timer_preinitialization_->cancel();
}

//}

/* initialize() //{ */

void MrsIcmImuDriver::initialize() {

  timer_imu_     = create_wall_timer(std::chrono::milliseconds(5000), std::bind(&MrsIcmImuDriver::timerImu, this));
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", 10);

  RCLCPP_INFO(node_->get_logger(), "Initialized");
  is_initialized_ = true;
}

//}

// | --------------------- timer callbacks -------------------- |

/* timerImu() //{ */

void MrsIcmImuDriver::timerImu() {
  if (!is_initialized_)
    return;

  FILE* fifo = fopen("/dev/icm_imu", "r");

  if (fifo == nullptr) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "Failed to open FIFO");
    return;
  }

  /* float    acc_x, acc_y, acc_z; */
  /* float    gyro_x, gyro_y, gyro_z; */
  /* uint64_t time_s, time_nsec; */

  sensor_msgs::msg::Imu imu;
  int ret = fscanf(fifo, "Ax: %lf Ay: %lf Az: %lf Gx: %lf Gy: %lf Gz: %lf Time: %u:%u", &imu.linear_acceleration.x, &imu.linear_acceleration.y,
                   &imu.linear_acceleration.z, &imu.angular_velocity.x, &imu.angular_velocity.y, &imu.angular_velocity.z, &imu.header.stamp.sec,
                   &imu.header.stamp.nanosec);

  if (ret == 8) {  // Check if all 8 values were successfully read
    try {
      RCLCPP_INFO_ONCE(node_->get_logger(), "Publishing IMU data");
      imu_publisher_->publish(imu);
    }
    catch (...) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "Exception caught when publishing");
    }

  } else {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Error reading from FIFO or no more data");
  }

  fclose(fifo);
  return;
}

//}

}  // namespace mrs_icm_imu_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_icm_imu_driver::MrsIcmImuDriver);
