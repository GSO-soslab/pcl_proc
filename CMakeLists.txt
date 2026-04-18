cmake_minimum_required(VERSION 3.5)
project(iceberg_nav)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ping360_msgs REQUIRED)
find_package(mvp_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Add executable target
add_executable(msis_pcl src/msis_pcl.cpp)

add_executable(msis_voxels src/msis_voxels.cpp)


# Link dependencies to the target
ament_target_dependencies(msis_pcl
  rclcpp
  std_msgs
  mvp_msgs
  cv_bridge
  image_transport
  sensor_msgs
  ping360_msgs
)

ament_target_dependencies(msis_voxels
  rclcpp
  std_msgs
  visualization_msgs
  sensor_msgs
  ping360_msgs
  geometry_msgs
)

# Link OpenCV separately
target_link_libraries(msis_pcl ${OpenCV_LIBRARIES})

# Install the target
install(TARGETS msis_pcl
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS msis_voxels
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(voxel_log_odds_visualizer src/voxel_log_odds_visualizer.cpp)
ament_target_dependencies(voxel_log_odds_visualizer
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  nav_msgs
)
target_include_directories(voxel_log_odds_visualizer PUBLIC ${EIGEN3_INCLUDE_DIR})
install(TARGETS voxel_log_odds_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  iceberg_nav/filter.py
  iceberg_nav/path_gen.py
  iceberg_nav/wp_admin.py
  iceberg_nav/loop.py
  iceberg_nav/msis_prob_clouds.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY iceberg_nav/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
ament_package()