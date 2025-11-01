cmake_minimum_required(VERSION 3.8)
project(autonomous_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =========================
# Install rules
# =========================
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# ... (ফাইলের উপরের অংশ যেমন আছে থাকবে)

install(
    DIRECTORY launch config urdf worlds maps
    DESTINATION share/${PROJECT_NAME}
)

# ... (ফাইলের বাকি অংশ যেমন আছে থাকবে)

ament_package()
