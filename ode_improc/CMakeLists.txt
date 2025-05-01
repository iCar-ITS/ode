cmake_minimum_required(VERSION 3.10)
project(ode_improc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

# include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# add executable for C++
add_executable(undistort_node src/undistort_node.cpp)# Require C99 and C++17
ament_target_dependencies(undistort_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge)

target_link_libraries(undistort_node
  ${OpenCV_LIBS}
  )

install(TARGETS undistort_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


# add executable for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/rectify_node_py.py
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
