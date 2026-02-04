# ROS2 LSM6DSV16X Driver
A ROS2 Jazzy driver for the [SparkFun 9DoF LSM6DSV16X IMU.](https://www.sparkfun.com/sparkfun-6dof-imu-breakout-lsm6dsv16x-qwiic.html)

### Getting Started:
Be sure to initialize or update the linked submodules with `git submodule --init --recursive` before running the node.

**Quickstart using the ROS2 CLI:** </br>
`ros2 run ros_imu_lsm6dsv16x lsm6dsv16x`

**Python Launch File:**
```python
    imu = Node(
        package="ros_imu_lsm6dsv16x",
        executable="lsm6dsv16x"
    )
```
### Parameters:
| Name | Default Value | Type |
|---|---|---|
| `dev_path` | /dev/i2c-7 | string |
| `frame_id` | imu | string |
| `topic` | /imu | string|
| `i2c_addr` | `0x6B`| int|
