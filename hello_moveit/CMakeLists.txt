cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

# hello_moveit executable
add_executable(hello_moveit src/hello_moveit.cpp)

target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  hello_moveit
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  control_msgs
)

# pick_place executable
add_executable(pick_place src/pick_place.cpp)

target_include_directories(pick_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(pick_place PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  pick_place
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  control_msgs
)

install(TARGETS
  hello_moveit
  pick_place
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()