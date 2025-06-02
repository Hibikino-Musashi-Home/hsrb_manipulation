cmake_minimum_required(VERSION 3.5)
project(hsrb_planner_plugins)

find_package(ament_cmake REQUIRED)
find_package(hsr_kinematics REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tmc_robot_planner REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/${PROJECT_NAME}/hsrb_plugins.cpp
  src/${PROJECT_NAME}/look_hand.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hsr_kinematics
  pluginlib
  rclcpp
  tmc_robot_planner
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)
  
  ament_add_gtest(look_hand-test test/look_hand-test.cpp)
  target_link_libraries(look_hand-test ${PROJECT_NAME})
  ament_target_dependencies(look_hand-test tmc_manipulation_tests)

endif()

pluginlib_export_plugin_description_file(tmc_robot_planner hsrb_plugins.xml)

install(
  TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hsr_kinematics
  pluginlib
  rclcpp
  tmc_robot_planner
)

ament_package()
