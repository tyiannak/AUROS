cmake_minimum_required(VERSION 2.8.3)
project(audio_features_classification)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(catkin REQUIRED COMPONENTS
  roslib
  std_msgs
  sensor_msgs
  message_runtime
  message_generation
)

add_message_files(FILES classificationResult.msg)

generate_messages(
	DEPENDENCIES
	std_msgs
)

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)

catkin_package(
   	LIBRARIES ${PROJECT_NAME}
  	CATKIN_DEPENDS roslib std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} src)

aux_source_directory(src SRC_LIST)

file(GLOB_RECURSE  LIB_CPP "./src/*.cpp")
file(GLOB_RECURSE  LIB_C "./src/*.c")
file(GLOB_RECURSE  LIB_HDRS "./src/*.h")

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall" )