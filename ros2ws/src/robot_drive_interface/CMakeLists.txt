cmake_minimum_required(VERSION 3.8)
project(robot_drive_interface)

add_library(shared_defs INTERFACE)
target_include_directories(shared_defs INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/include
)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)


add_executable(drive_node src/drive_node.cpp)
target_include_directories(drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(drive_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  drive_node
  robot_msgs
  "rclcpp"
)

add_executable(video_publisher src/video_publisher.cpp)
target_include_directories(video_publisher PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(video_publisher PUBLIC c_std_99 cxx_std_17)
find_package(sensor_msgs)
find_package(cv_bridge)

find_package(OpenCV REQUIRED)

ament_target_dependencies(
        video_publisher
        rclcpp
        sensor_msgs
        cv_bridge
)
target_link_libraries(video_publisher
        ${OpenCV_LIBS}
)


target_link_libraries(drive_node shared_defs)


install(TARGETS drive_node video_publisher
  DESTINATION lib/${PROJECT_NAME})

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
