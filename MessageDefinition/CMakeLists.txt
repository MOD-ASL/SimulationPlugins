cmake_minimum_required(VERSION 2.8)
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  collision_message.proto
  command_message.proto
  config_message.proto
  gait_recorder_message.proto
  ${PROTOBUF_IMPORT_DIRS}/pose.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(collision_message SHARED ${PROTO_SRCS})
target_link_libraries(collision_message ${PROTOBUF_LIBRARY})

add_library(command_message SHARED ${PROTO_SRCS})
target_link_libraries(command_message ${PROTOBUF_LIBRARY})

add_library(config_message SHARED ${PROTO_SRCS})
target_link_libraries(config_message ${PROTOBUF_LIBRARY})

add_library(gait_recorder_message SHARED ${PROTO_SRCS})
target_link_libraries(gait_recorder_message ${PROTOBUF_LIBRARY})