#include <cstdio>
#include <string>
#include <string_view>

#include <fcntl.h>
#include <sys/ioctl.h>

#include "i2c/i2c.h"
#include "rclcpp/rclcpp.hpp"

extern "C" {
#include "driver/lsm6dsv16x_reg.h"
}

constexpr const char* i2c_dev_addr = "/dev/i2c-0";

extern "C" {
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  if (handle == nullptr || bufp == nullptr) { return -1; }
  I2CDevice* i2c_dev = reinterpret_cast<I2CDevice*>(handle);

  if (i2c_write(i2c_dev, reg, bufp, len) < 0) {
    return -1;
  }

  return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  if (handle == nullptr || bufp == nullptr) { return -1; }
  I2CDevice* i2c_dev = reinterpret_cast<I2CDevice*>(handle);

  if (i2c_read(i2c_dev, reg, bufp, len) < 0)
  {
    return -1;
  }

  return 0;
}

void platform_delay(uint32_t millisec) {
  struct timespec timespec;

  timespec.tv_sec = millisec / 1000;
  timespec.tv_nsec = (millisec % 1000) * 1000000;
  
  int res;
  do { 
    res = nanosleep(&timespec, &timespec);
  } while (res && errno == EINTR);
}
}

class LSM6DSV16XNode : public rclcpp::Node {
  public:
    I2CDevice i2c_dev;
    stmdev_ctx_t dev_ctx;

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

      dev_ctx.write_reg = platform_write;
      dev_ctx.read_reg = platform_read;
      dev_ctx.mdelay = platform_delay;
      dev_ctx.handle = &i2c_dev;

      uint8_t device_id;
      lsm6dsv16x_device_id_get(&dev_ctx, &device_id);
      if (device_id != LSM6DSV16X_ID)
      {
        std::cout << "Device ID " << device_id << "did not match expected LSM6 ID of " << LSM6DSV16X_ID << std::endl;
        i2c_close(i2c_dev.bus);
        exit(1);
      }

      // Reset LSM6
      lsm6dsv16x_sw_por(&dev_ctx);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  return 0;
}
