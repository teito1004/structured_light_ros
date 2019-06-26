cmake_minimum_required(VERSION 2.8.3)
project(phaseshift)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
#  opencv2
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cv_bridge_tutorial
#  CATKIN_DEPENDS cv_bridge opencv2 rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(color_capture src/color_capture.cpp)
add_executable(projcalib src/main_projcalib.cpp)

add_executable(gray_capture src/gray_capture.cpp)
add_executable(gray_decode src/gray_decode.cpp)
add_executable(triangulation src/triangulation.cpp)

target_link_libraries(color_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(projcalib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

target_link_libraries(gray_capture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(gray_decode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(triangulation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
