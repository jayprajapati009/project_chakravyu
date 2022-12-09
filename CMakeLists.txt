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

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(roomba_algo src/cleaner2.cpp)
ament_target_dependencies(roomba_algo rclcpp std_msgs sensor_msgs geometry_msgs)
add_library(robot_library src/robot.cpp)
target_include_directories(robot_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
ament_target_dependencies(robot_library rclcpp std_msgs sensor_msgs geometry_msgs)


target_link_directories(roomba_algo PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(roomba_algo robot_library)

install(
  DIRECTORY include/project_chakravyu
  DESTINATION include
)


install(TARGETS
  roomba_algo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  models
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}/
)


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
