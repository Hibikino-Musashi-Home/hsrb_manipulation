cmake_minimum_required(VERSION 3.8)
project(hsrb_ik_solver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hsrb_analytic_ik REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tmc_ik_solver_node REQUIRED)

add_library(${PROJECT_NAME} SHARED src/ik_solver_plugin.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  hsrb_analytic_ik
  pluginlib
  rclcpp
  tmc_ik_solver_node
)

pluginlib_export_plugin_description_file(tmc_ik_solver_node ik_solver_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY example/
  DESTINATION share/${PROJECT_NAME}/example
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(test_hsr_ik_solver_plugin test/test_ik_solver_plugin.cpp)
  ament_target_dependencies(test_hsr_ik_solver_plugin tf2_eigen tmc_manipulation_tests)
  target_link_libraries(test_hsr_ik_solver_plugin ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  hsrb_analytic_ik
  rclcpp
  tmc_ik_solver_node
)

ament_export_libraries(${PROJECT_NAME})

ament_package()

