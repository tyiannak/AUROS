cmake_minimum_required(VERSION 2.8.3)
project(audio_features_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
	FILES
	featMsg.msg
	classificationResult.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime message_generation std_msgs 
)

