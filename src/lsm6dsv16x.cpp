#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <ranges>
#include <ratio>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>

#include "geometry_msgs/msg/quaternion.hpp"
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

[[nodiscard]] double f16_to_double(uint16_t h) {
  union {
    float_t ret;
    uint32_t retbits;
  } conv;
  conv.retbits = lsm6dsv16x_from_f16_to_f32(h);

  return static_cast<double>(conv.ret);
}

geometry_msgs::msg::Quaternion sflp_to_quaternion(uint16_t sflp[3]) {
  geometry_msgs::msg::Quaternion ret;

  ret.x = f16_to_double(sflp[0]);
  ret.y = f16_to_double(sflp[1]);
  ret.z = f16_to_double(sflp[2]);

  double sum_squares = (ret.x * ret.x) + (ret.y * ret.y) + (ret.z * ret.z);

  if (sum_squares > 1.0) {
    float_t n = std::sqrt(sum_squares);
    ret.x /= n;
    ret.y /= n;
    ret.z /= n;
    sum_squares = 1.0;
  }

  ret.w = std::sqrt(1.0 - sum_squares);

  return ret;
}

class LSM6DSV16XNode : public rclcpp::Node {
public:
  LSM6DSV16XNode() : Node("lsm6dsv16x_node") {
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

    configure_imu();

    ros_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Started LSM6DSV16X IMU Node");

    ros_timer = this->create_wall_timer(std::chrono::milliseconds(100), [this] {
      ros_publisher->publish(make_imu_message());
    });
  }

  ~LSM6DSV16XNode() { i2c_close(i2c_dev.bus); }

private:
  I2CDevice i2c_dev;
  stmdev_ctx_t dev_ctx;
  rclcpp::TimerBase::SharedPtr ros_timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_publisher;

  void configure_imu() {
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

    // Set number of samples stored in FIFO, we should only need 1 for
    // our purposes?
    lsm6dsv16x_fifo_watermark_set(&dev_ctx, 1);

    // Enable only quaternion for SFLP output
    lsm6dsv16x_fifo_sflp_raw_t fifo_sflp{
        .game_rotation = 1, .gravity = 0, .gbias = 0};
    lsm6dsv16x_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);

    // Set FIFO to continuous mode to always overwrite the oldest sample
    lsm6dsv16x_fifo_mode_set(&dev_ctx, LSM6DSV16X_STREAM_MODE);

    // Stop on WTM, we only want the most recent sample
    lsm6dsv16x_fifo_stop_on_wtm_set(&dev_ctx, LSM6DSV16X_FIFO_EV_WTM);

    // Set output data rates
    lsm6dsv16x_xl_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
    lsm6dsv16x_gy_data_rate_set(&dev_ctx, LSM6DSV16X_ODR_AT_120Hz);
    lsm6dsv16x_sflp_data_rate_set(&dev_ctx, LSM6DSV16X_SFLP_120Hz);

    lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

    // TODO: Do we want/need to add a GBias for quaternion?
    // lsm6dsv16x_sflp_game_gbias_set()
  }

  std::unique_ptr<sensor_msgs::msg::Imu> make_imu_message() {
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

    lsm6dsv16x_fifo_status_t fifo_status;
    lsm6dsv16x_fifo_status_get(&dev_ctx, &fifo_status);
    bool got_orientation = false;
    if (fifo_status.fifo_th) {
      // There is a sample present
      lsm6dsv16x_fifo_out_raw_t raw_fifo_data;
      lsm6dsv16x_fifo_out_raw_get(&dev_ctx, &raw_fifo_data);

      if (raw_fifo_data.tag == LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG) {
        message->orientation = sflp_to_quaternion(
            reinterpret_cast<uint16_t *>(raw_fifo_data.data));
      }
    }

    if (!got_orientation) {
      // No orientation present, set first covariance element to -1 to
      // indicate as such
      message->orientation_covariance[0] = -1.f;
      message->orientation.w = 0;
      message->orientation.x = 0;
      message->orientation.y = 0;
      message->orientation.z = 0;
    }

    return message;
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
