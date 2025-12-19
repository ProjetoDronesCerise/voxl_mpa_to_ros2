cmake_minimum_required(VERSION 3.5)
project(sentinel_rtsp_relay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTRTSP REQUIRED gstreamer-rtsp-server-1.0)

add_executable(rtsp_relay_node src/rtsp_relay_node.cpp)

target_include_directories(rtsp_relay_node PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GSTAPP_INCLUDE_DIRS}
  ${GSTRTSP_INCLUDE_DIRS}
)

target_link_libraries(rtsp_relay_node
  ${GST_LIBRARIES}
  ${GSTAPP_LIBRARIES}
  ${GSTRTSP_LIBRARIES}
)


target_compile_options(rtsp_relay_node PRIVATE -Wno-error=deprecated-declarations)

ament_target_dependencies(rtsp_relay_node rclcpp sensor_msgs)

install(TARGETS rtsp_relay_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
