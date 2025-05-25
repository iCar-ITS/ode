cmake_minimum_required(VERSION 3.8)
project(ode_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(pcl2depth src/pcl2depth.cpp)

ament_target_dependencies(pcl2depth
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  image_geometry
  tf2
)

add_executable(pcl2depth2 src/pcl2depth2.cpp)

ament_target_dependencies(pcl2depth2
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  image_geometry
  tf2
)

target_link_libraries(pcl2depth
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp::yaml-cpp)

target_link_libraries(pcl2depth2
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp::yaml-cpp)

target_include_directories(pcl2depth PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


add_executable(pcl_concat src/pcl_concat.cpp)

ament_target_dependencies(pcl_concat
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  message_filters
)

install(TARGETS pcl2depth pcl_concat pcl2depth2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
