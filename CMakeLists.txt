cmake_minimum_required(VERSION 2.8.3)
project(ros_opencv)
 
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  )
 
find_package( OpenCV REQUIRED )
 
catkin_package()
 
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
 
add_executable(ros_opencv src/face_test.cpp)
 
target_link_libraries(ros_opencv
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
 
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
