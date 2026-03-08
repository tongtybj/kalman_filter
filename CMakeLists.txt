cmake_minimum_required(VERSION 3.5)
project(kalman_filter)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_library(kf_pos_vel_acc_plugin SHARED
  src/kf_pos_vel_acc_plugin.cpp
)
target_include_directories(kf_pos_vel_acc_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(kf_pos_vel_acc_plugin
  Eigen3::Eigen
)
ament_target_dependencies(kf_pos_vel_acc_plugin
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(
  kalman_filter
  kf_plugins.xml
)
install(
  TARGETS kf_pos_vel_acc_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(lpf_test
  src/lpf_test.cpp
)
target_link_libraries(lpf_test
  Eigen3::Eigen
)
install(
  TARGETS lpf_test
  DESTINATION lib/${PROJECT_NAME}
  )

add_executable(kf_plugin_test src/kf_plugin_test.cpp)
target_link_libraries(kf_plugin_test
  Eigen3::Eigen
)
ament_target_dependencies(kf_plugin_test
  rclcpp
  pluginlib
  )

ament_export_include_directories(include)

install(
  TARGETS kf_plugin_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES kf_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
