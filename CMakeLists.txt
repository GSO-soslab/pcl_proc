cmake_minimum_required(VERSION 3.5)
project(pcl_proc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ping360_msgs REQUIRED)
find_package(mvp_msgs REQUIRED)
find_package(OpenCV REQUIRED)

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

# Link OpenCV separately
target_link_libraries(msis_pcl ${OpenCV_LIBRARIES})

# Install the target
install(TARGETS msis_pcl
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  pcl_proc/filter.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  pcl_proc/path_gen.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  pcl_proc/wp_admin.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY pcl_proc/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
ament_package()