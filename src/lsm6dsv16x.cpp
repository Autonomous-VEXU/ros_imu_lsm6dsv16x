#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <ranges>
#include <ratio>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <thread>

#include "geometry_msgs/msg/quaternion.hpp"
#include "i2c/i2c.h"
#include "lsm6dsv16x_cfg.hpp"
#include "lsm6dsv16x_reg.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {

/**
 * @brief Milli-degrees per second to radians per second
 *
 * @param mdps milli-degrees per second
 * @return radians per second
 */
[[nodiscard]] constexpr double mdps_to_rad_per_sec(float mdps) {
  return (static_cast<double>(mdps) / 1000.) / ((2 * std::numbers::pi) / 360);
}

/**
 * @brief Milli-G's (linear acceleration) to m/s^2
 *
 * @param mG Milli-G's
 * @return meters per second-squared
 */
[[nodiscard]] constexpr double mG_to_mps2(float mG) {
  return (static_cast<double>(mG) / 1000.) / 9.81;
}

/**
 * @brief Converts 16-bit float to a double (64-bit float)
 *
 * @param f16 16-bit float input
 * @return 64 bit float output
 */
[[nodiscard]] double f16_to_double(uint16_t f16) {
  union {
    float ret;
    uint32_t retbits;
  } conv;
  // Type-punned u32 to float for some reason, thanks ST
  conv.retbits = lsm6dsv16x_from_f16_to_f32(f16);

  return static_cast<double>(conv.ret);
}

/**
 * @brief Convert the LSM6's SFLP "game vector" output of 3x f16's into a
 * quaternion
 *
 * @param sflp Reading from LSM6's SFLP "game vector"
 * @return Equivalent quaternion
 */
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

double raw_gyro_to_rad_per_sec(uint16_t raw_gyro) {
  float mdps;
  if (IMU_GYRO_SCALE == LSM6DSV16X_125dps) {
    mdps = lsm6dsv16x_from_fs125_to_mdps(raw_gyro);
  } else if (IMU_GYRO_SCALE == LSM6DSV16X_250dps) {
    mdps = lsm6dsv16x_from_fs250_to_mdps(raw_gyro);
  } else if (IMU_GYRO_SCALE == LSM6DSV16X_500dps) {
    mdps = lsm6dsv16x_from_fs500_to_mdps(raw_gyro);
  } else if (IMU_GYRO_SCALE == LSM6DSV16X_1000dps) {
    mdps = lsm6dsv16x_from_fs1000_to_mdps(raw_gyro);
  } else if (IMU_GYRO_SCALE == LSM6DSV16X_2000dps) {
    mdps = lsm6dsv16x_from_fs2000_to_mdps(raw_gyro);
  } else if (IMU_GYRO_SCALE == LSM6DSV16X_4000dps) {
    mdps = lsm6dsv16x_from_fs4000_to_mdps(raw_gyro);
  } else {
    return FP_NAN;
  }

  return mdps_to_rad_per_sec(mdps);
}

double raw_accel_to_mps2(uint16_t raw_accel) {
  float mg;
  if (IMU_ACCEL_SCALE == LSM6DSV16X_2g) {
    mg = lsm6dsv16x_from_fs2_to_mg(raw_accel);
  } else if (IMU_ACCEL_SCALE == LSM6DSV16X_4g) {
    mg = lsm6dsv16x_from_fs4_to_mg(raw_accel);
  } else if (IMU_ACCEL_SCALE == LSM6DSV16X_8g) {
    mg = lsm6dsv16x_from_fs8_to_mg(raw_accel);
  } else if (IMU_ACCEL_SCALE == LSM6DSV16X_16g) {
    mg = lsm6dsv16x_from_fs16_to_mg(raw_accel);
  } else {
    return FP_NAN;
  }

  return mG_to_mps2(mg);
}

} // namespace

class LSM6DSV16XNode : public rclcpp::Node {
public:
  LSM6DSV16XNode() : Node("lsm6dsv16x_node") {
    int i2c_fd = i2c_open(I2C_DEVICE_PATH);
    if (i2c_fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not open i2c device");
      return;
    }

    i2c_dev.bus = i2c_fd;
    i2c_dev.addr = IMU_I2C_ADDRESS;
    i2c_dev.iaddr_bytes = 1;
    i2c_dev.page_bytes = 256;

    dev_ctx.write_reg = i2c_write_impl;
    dev_ctx.read_reg = i2c_read_impl;
    dev_ctx.mdelay = i2c_ms_delay_impl;
    dev_ctx.handle = &i2c_dev;

    configure_imu();

    ros_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
        ROS_TOPIC_NAME, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Started LSM6DSV16X IMU Node");

    ros_timer = this->create_wall_timer(
        PUBLISH_PERIOD, [this] { ros_publisher->publish(make_imu_message()); });
  }

  ~LSM6DSV16XNode() { i2c_close(i2c_dev.bus); }

private:
  I2CDevice i2c_dev;
  stmdev_ctx_t dev_ctx;
  rclcpp::TimerBase::SharedPtr ros_timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_publisher;

  /**
   * @brief Configure the IMU's registers as needed. See datasheet and
   * driver source code for more details.
   */
  void configure_imu() {
    uint8_t device_id;
    lsm6dsv16x_device_id_get(&dev_ctx, &device_id);
    if (device_id != LSM6DSV16X_ID) {
      RCLCPP_ERROR(this->get_logger(),
                   "Device ID 0x%02X did not math expected LSM6 ID of 0x%02X",
                   device_id, LSM6DSV16X_ID);
      i2c_close(i2c_dev.bus);
    }

    // Reset LSM6
    lsm6dsv16x_sw_por(&dev_ctx);
    // Enable Block Data Update
    lsm6dsv16x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    lsm6dsv16x_gy_full_scale_set(&dev_ctx, IMU_GYRO_SCALE);
    lsm6dsv16x_xl_full_scale_set(&dev_ctx, IMU_ACCEL_SCALE);

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
    lsm6dsv16x_xl_data_rate_set(&dev_ctx, IMU_GYRO_ACCEL_FREQ);
    lsm6dsv16x_gy_data_rate_set(&dev_ctx, IMU_GYRO_ACCEL_FREQ);
    lsm6dsv16x_sflp_data_rate_set(&dev_ctx, IMU_SFLP_FREQ);

    lsm6dsv16x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

    RCLCPP_INFO(this->get_logger(), "Configured LSM6DSV16X IMU successfully");

    // TODO: Do we want/need to add a GBias for quaternion?
    // lsm6dsv16x_sflp_game_gbias_set()
  }

  // Read data from the IMU and assemble it into a ROS message
  std::unique_ptr<sensor_msgs::msg::Imu> make_imu_message() {
    auto message = std::make_unique<sensor_msgs::msg::Imu>();
    message->header.stamp = this->get_clock()->now();
    message->header.frame_id = ROS_FRAME_ID;

    std::array<int16_t, 3> gyro_raw;
    lsm6dsv16x_angular_rate_raw_get(&dev_ctx, gyro_raw.data());
    message->angular_velocity.x = raw_gyro_to_rad_per_sec(gyro_raw[0]);
    message->angular_velocity.y = raw_gyro_to_rad_per_sec(gyro_raw[1]);
    message->angular_velocity.z = raw_gyro_to_rad_per_sec(gyro_raw[2]);

    std::array<int16_t, 3> accel_raw;
    lsm6dsv16x_acceleration_raw_get(&dev_ctx, accel_raw.data());
    message->linear_acceleration.x = raw_accel_to_mps2(accel_raw[0]);
    message->linear_acceleration.y = raw_accel_to_mps2(accel_raw[1]);
    message->linear_acceleration.z = raw_accel_to_mps2(accel_raw[2]);

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

  /**
   * @brief Write data through the LSM6 driver onto the I2C bus.
   *        Only called via callback in the LSM6 driver.
   *
   * @param handle I2CDevice handle
   * @param reg Register to write to
   * @param bufp Pointer to data to write
   * @param len Length of data
   * @return 0 for OK, -1 for error
   */
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

  /**
   * @brief Read data through the LSM6 driver onto the I2C bus.
   *        Only called via callback in the LSM6 driver.
   *
   * @param handle I2CDevice handle
   * @param reg Register to read from
   * @param bufp Pointer to data to read to
   * @param len Length of data to read
   * @return 0 for OK, -1 for error
   */
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

  /**
   * @brief Delay for some number of milliseconds. Only called by the
   *        LSM6's driver
   *
   * @param millisec Number of milliseconds
   */
  static void i2c_ms_delay_impl(uint32_t millisec) {
    std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LSM6DSV16XNode>());
  rclcpp::shutdown();
}
