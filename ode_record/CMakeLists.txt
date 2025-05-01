cmake_minimum_required(VERSION 3.8)
project(ode_record)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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
find_package(yaml-cpp REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})



add_executable(record src/record.cpp)
target_include_directories(record PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(record PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(record
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  pcl_ros
  cv_bridge)

target_link_libraries(record
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS})



add_executable(replay src/play.cpp)
target_compile_features(replay PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(replay
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  pcl_ros
  cv_bridge)

target_link_libraries(replay
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp::yaml-cpp)




install(TARGETS record replay
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  )

  install(PROGRAMS scripts/telemetry.py DESTINATION lib/${PROJECT_NAME})

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
