cmake_minimum_required(VERSION 3.8)
project(project_chakravyu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest)

# Coverage
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
  set(ARG_CONFIG_FILE "${PROJECT_SOURCE_DIR}/uncrustify_google_cpp_allman.cfg"
    CACHE STRING "uncrustify config file")
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

# add_executable(slave src/slave.cpp)
# ament_target_dependencies(slave tf2 tf2_ros rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)
add_executable(master src/main.cpp)
ament_target_dependencies(master tf2 tf2_ros rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)
add_library(robot_library src/robot.cpp src/master.cpp)
target_include_directories(robot_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
ament_target_dependencies(robot_library tf2 tf2_ros rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

target_link_directories(master PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(master robot_library)

# target_link_directories(slave PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
# target_link_libraries(slave robot_library)
install(
  DIRECTORY include/project_chakravyu
  DESTINATION include
)

install(TARGETS
  master

  # slave
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  models
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)

  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  set(TESTFILES
    test/test.cpp)
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs)

  install(TARGETS
    test_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})
endif()

# # Install some arbitrary executables
install(PROGRAMS # want to make the file executable (ie. chmod a+x)
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()
