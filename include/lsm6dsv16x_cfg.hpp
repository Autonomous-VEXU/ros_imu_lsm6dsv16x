#pragma once
#include <chrono>

#include "lsm6dsv16x_reg.h"

using namespace std::chrono_literals;

/**
 * @brief How often the node will read and publish the IMU data
 */
constexpr std::chrono::duration PUBLISH_PERIOD = 100ms;

/**
 * @brief Linux device path to the i2c bus
 */
constexpr const char *I2C_DEVICE_PATH = "/dev/i2c-0";

/**
 * @brief Topic name that the node will publish on
 */
constexpr const char *ROS_TOPIC_NAME = "imu/data";

/**
 * @brief Frame ID that the node will publish with
 */
constexpr const char *ROS_FRAME_ID = "lsm6dsv16x_imu";

/**
 * @brief The device I2C address of the IMU chip. Should probably be
 * either LSM6DSV16X_I2C_ADD_L or LSM6DSV16X_I2C_ADD_H, unless you have
 * an I2C address translator or multiplexer.
 */
constexpr uint8_t IMU_I2C_ADDRESS = LSM6DSV16X_I2C_ADD_L;

/**
 * @brief How frequently the IMU chip is configured to sample data.
 *
 * @note Recommended to have the SFLP sample at the same rate as the gyro
 * and accelerometer.
 */
constexpr lsm6dsv16x_data_rate_t IMU_GYRO_ACCEL_FREQ = LSM6DSV16X_ODR_AT_120Hz;
constexpr lsm6dsv16x_sflp_data_rate_t IMU_SFLP_FREQ = LSM6DSV16X_SFLP_120Hz;

/**
 * @brief Maximum reading scales. Choose depending on your max speed.
 */
constexpr lsm6dsv16x_gy_full_scale_t IMU_GYRO_SCALE = LSM6DSV16X_1000dps;
constexpr lsm6dsv16x_xl_full_scale_t IMU_ACCEL_SCALE = LSM6DSV16X_4g;
