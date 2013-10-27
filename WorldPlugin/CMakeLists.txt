cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(
	${GAZEBO_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}/msgs
	)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_library(SuperServer SHARED SuperServer.cc)
target_link_libraries(SuperServer collision_message_plus ${GAZEBO_libraries} ${Boost_LIBRARIES})
add_dependencies(SuperServer collision_message_plus)
