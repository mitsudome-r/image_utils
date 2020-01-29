cmake_minimum_required(VERSION 2.8.3)
project(image_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_utils
#  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gain_bias_filter src/gain_bias_filter.cpp)
add_dependencies(gain_bias_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gain_bias_filter
  ${catkin_LIBRARIES}
)
add_executable(camera_info_publisher src/camera_info_publisher.cpp)
add_dependencies(camera_info_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_info_publisher
  ${catkin_LIBRARIES}
)
add_executable(camera_info_publisher_synchronized src/camera_info_publisher_synchronized.cpp)
add_dependencies(camera_info_publisher_synchronized ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_info_publisher_synchronized
  ${catkin_LIBRARIES}
)

install(TARGETS gain_bias_filter camera_info_publisher camera_info_publisher_synchronized
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    launch/camera_info_publisher.launch
    launch/camera_info_publisher_synchronized.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
