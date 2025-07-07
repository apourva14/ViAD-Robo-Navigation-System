cmake_minimum_required(VERSION 3.8)
project(rav_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)

# install non-Python assets
install(DIRECTORY
  rviz
  urdf
  launch
  world
  model
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# install your Python scripts as executables
install(PROGRAMS
  script/joystick_control.py
  script/point_follow.py
  script/multipoint_follow.py
  script/ultrasonic_sensor_obstacle_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
