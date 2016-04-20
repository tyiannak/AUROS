cmake_minimum_required(VERSION 2.8.3)
project(audio_features_extraction)



set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
)

add_message_files(
	FILES
	featMsg.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
find_package(portaudiocpp)
find_package(fftw3)
find_package(sndfile)

catkin_package(
  CATKIN_DEPENDS roscpp rospy roslib std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIR} ${Boost_INCLUDE_DIR} src)

aux_source_directory(src SRC_LIST)

add_executable(audio_features_extraction ${SRC_LIST})
target_link_libraries( ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} portaudiocpp fftw3 sndfile)

file(GLOB_RECURSE  LIB_CPP "./src/*.cpp")
file(GLOB_RECURSE  LIB_HDRS "./src/*.h")

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall" )

#add_library(audio_features_extraction ${})