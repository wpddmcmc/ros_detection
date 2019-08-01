cmake_minimum_required(VERSION 2.8.3)
project(detector)

# using C++11 
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")

# if u have OpenCV version more than one, set the build path which one u want to use
#set(OpenCV_DIR "{YourPath}")
 
find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  roscpp
  roslib
  std_msgs
  message_generation
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
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/talker
DIR_SRCS)

add_executable(${PROJECT_NAME}_node ${DIR_SRCS})

add_executable(listener ${CMAKE_CURRENT_SOURCE_DIR}/src/listener/listener.cpp)

target_link_libraries( ${PROJECT_NAME}_node 
${OpenCV_LIBS}
${catkin_LIBRARIES})

target_link_libraries(listener ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node  detector_generate_messages_cpp)
