#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <ranges>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>

#include "i2c/i2c.h"
#include "lsm6dsv16x_reg.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

constexpr const char *i2c_dev_addr = "/dev/i2c-0";

[[nodiscard]] constexpr double mdps_to_rad_per_sec(double mdps) {
  return (mdps / 1000.) / ((2 * std::numbers::pi) / 360);
}

[[nodiscard]] constexpr double mG_to_meter_per_sec2(double mG) {
  return (mG / 1000.) / 9.81;
}

geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch,
                                                   double yaw) {
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

class LSM6DSV16XNode : public rclcpp::Node {
public:
  LSM6DSV16XNode() : Node("lsm6dsv16x_node"), count(0) {
    int i2c_fd = i2c_open(i2c_dev_addr);
    if (i2c_fd < 0) {
      std::cout << "Could not open i2c device" << std::endl;
      exit(1);
    }

    i2c_dev.bus = i2c_fd;
    i2c_dev.addr = LSM6DSV16X_I2C_ADD_L;
    i2c_dev.iaddr_bytes = 1;
    i2c_dev.page_bytes = 256;

    dev_ctx.write_reg = i2c_write_impl;
    dev_ctx.read_reg = i2c_read_impl;
    dev_ctx.mdelay = i2c_ms_delay_impl;
    dev_ctx.handle = &i2c_dev;

    uint8_t device_id;
    lsm6dsv16x_device_id_get(&dev_ctx, &device_id);
    if (device_id != LSM6DSV16X_ID) {
      std::cout << "Device ID " << device_id
                << "did not match expected LSM6 ID of " << LSM6DSV16X_ID
                << std::endl;
      i2c_close(i2c_dev.bus);
      exit(1);
    }

    // Reset LSM6
    lsm6dsv16x_sw_por(&dev_ctx);
    /* Enable Block Data Update */
    lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // Set +/- 1000 deg/sec maximum gyroscope scale
    // If you change this, you must also change any use of
    // lsm6dsv16x_from_fs1000_to_mdps()
    lsm6dsv16x_gy_full_scale_set(&dev_ctx, LSM6DSV16X_1000dps);

    // Set +/- 4G maximum linear acceleration scale
    // If you change this, you must also change any use of
    // lsm6dsv16x_from_fs4_to_mg()
    lsm6dsv16x_xl_full_scale_set(&dev_ctx, LSM6DSV16X_4g);
  }

private:
  I2CDevice i2c_dev;
  stmdev_ctx_t dev_ctx;
  size_t count;

  void make_imu_message() {
    auto message = std::make_unique<sensor_msgs::msg::Imu>();
    message->header.stamp = this->get_clock()->now();
    message->header.frame_id = "imu_lsm6dsv16x";

    std::array<int16_t, 3> angular_vel_raw;
    lsm6dsv16x_angular_rate_raw_get(&dev_ctx, angular_vel_raw.data());
    message->angular_velocity.x =
        mdps_to_rad_per_sec(lsm6dsv16x_from_fs1000_to_mdps(angular_vel_raw[0]));
    message->angular_velocity.y =
        mdps_to_rad_per_sec(lsm6dsv16x_from_fs1000_to_mdps(angular_vel_raw[1]));
    message->angular_velocity.z =
        mdps_to_rad_per_sec(lsm6dsv16x_from_fs1000_to_mdps(angular_vel_raw[2]));

    std::array<int16_t, 3> lin_accel_raw;
    lsm6dsv16x_acceleration_raw_get(&dev_ctx, lin_accel_raw.data());
    message->linear_acceleration.x =
        mG_to_meter_per_sec2(lsm6dsv16x_from_fs8_to_mg(lin_accel_raw[0]));
    message->linear_acceleration.y =
        mG_to_meter_per_sec2(lsm6dsv16x_from_fs8_to_mg(lin_accel_raw[1]));
    message->linear_acceleration.z =
        mG_to_meter_per_sec2(lsm6dsv16x_from_fs8_to_mg(lin_accel_raw[2]));

    count++;
  }

  static int32_t i2c_write_impl(void *handle, uint8_t reg, const uint8_t *bufp,
                                uint16_t len) {
    if (handle == nullptr || bufp == nullptr) {
      return -1;
    }
    I2CDevice *i2c_dev = reinterpret_cast<I2CDevice *>(handle);

    if (i2c_ioctl_write(i2c_dev, reg, bufp, len) < 0) {
      return -1;
    }

    return 0;
  }

  static int32_t i2c_read_impl(void *handle, uint8_t reg, uint8_t *bufp,
                               uint16_t len) {
    if (handle == nullptr || bufp == nullptr) {
      return -1;
    }
    I2CDevice *i2c_dev = reinterpret_cast<I2CDevice *>(handle);

    if (i2c_ioctl_read(i2c_dev, reg, bufp, len) < 0) {
      return -1;
    }

    return 0;
  }

  static void i2c_ms_delay_impl(uint32_t millisec) {
    struct timespec timespec;

    timespec.tv_sec = millisec / 1000;
    timespec.tv_nsec = (millisec % 1000) * 1000000;

    int res;
    do {
      res = nanosleep(&timespec, &timespec);
    } while (res && errno == EINTR);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  return 0;
}
