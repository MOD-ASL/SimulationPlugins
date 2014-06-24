# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
set (CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(
	${GAZEBO_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}/MessageDefinition
	)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/MessageDefinition)
add_subdirectory(../MessageDefinition MessageDefinition)

add_library(ModuleController SHARED ModuleController.cc)
target_link_libraries(ModuleController collision_message command_message ${GAZEBO_libraries} ${Boost_LIBRARIES})
add_dependencies(ModuleController collision_message command_message)
