cmake_minimum_required(VERSION 3.8)
project(ode_label)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(boundingboxes REQUIRED)
find_package(OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

add_executable(labeling src/labeling.cpp)
target_link_libraries(labeling
  ${OpenCV_LIBS})
target_include_directories(labeling PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(labeling PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(labeling
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  message_filters
  boundingboxes)

install(TARGETS labeling
  DESTINATION lib/${PROJECT_NAME})

ament_package()
