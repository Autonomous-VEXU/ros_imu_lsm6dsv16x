cmake_minimum_required(VERSION 3.8)
project(lsm6dsv16x)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)

add_executable(lsm6dsv16x src/lsm6dsv16x.cpp
                          src/driver/lsm6dsv16x_reg.c
                          lib/libi2c/src/i2c.c)
ament_target_dependencies(lsm6dsv16x rclcpp)
target_include_directories(lsm6dsv16x PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/lsm6dsv16x
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libi2c/include)
target_compile_features(lsm6dsv16x PUBLIC c_std_99 cxx_std_20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

install(TARGETS lsm6dsv16x
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
