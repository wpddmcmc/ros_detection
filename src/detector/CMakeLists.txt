cmake_minimum_required(VERSION 2.8.3)
project(detector)
 
# using C++11 
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")

# if u have OpenCV version more than one, set the build path which one u want to use
set(OpenCV_DIR "/home/mingcongchen/app/opencv-3.4.6/build")
 

find_package(catkin REQUIRED COMPONENTS
OpenCV REQUIRED
roscpp
roslib
std_msgs
message_generation
cv_bridge
image_transport
)

add_message_files(
  FILES
  Detector_Info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport message_runtime
)
 
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  /home/mingcongchen/app/caffe-ssd/include
  ${CMAKE_CURRENT_SOURCE_DIR}/detector_node/include
  ${CMAKE_CURRENT_SOURCE_DIR}/caffe_node/include
  /usr/local/cuda-9.0/include   
  include
  ${catkin_INCLUDE_DIRS}
)
 
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/detector_node/src/ detector_SRCS)
add_executable(detector_node ${detector_SRCS})
target_link_libraries(detector_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/stream_node/src/ stream_SRCS)
add_executable(stream_node ${stream_SRCS})
target_link_libraries(stream_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/caffe_node/src/ caffe_SRCS)
add_executable(caffe_node ${caffe_SRCS})
target_link_libraries(caffe_node 
${catkin_LIBRARIES} 
${OpenCV_LIBS}
/home/mingcongchen/app/caffe-ssd/build/lib/libcaffe.so
/usr/local/lib/libglog.so.0
/usr/local/lib/libboost_system.so.1.66.0)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/data_listener/src/ listener_SRCS)
add_executable(listener_node ${listener_SRCS})
target_link_libraries(listener_node ${catkin_LIBRARIES})