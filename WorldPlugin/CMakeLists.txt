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
	${CMAKE_CURRENT_BINARY_DIR}/../../Libraries/
	${CMAKE_CURRENT_BINARY_DIR}/../
	)
# Include all folders in Util folder
FILE(GLOB _ALL_FILES ${CMAKE_CURRENT_BINARY_DIR}/../../Util/ ${CMAKE_CURRENT_BINARY_DIR}/../../Util/*)
FOREACH(_FILE ${_ALL_FILES})
  IF(IS_DIRECTORY ${_FILE})
    INCLUDE_DIRECTORIES(${_FILE})
  ENDIF()
ENDFOREACH()
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/MessageDefinition)
add_subdirectory(../MessageDefinition MessageDefinition)

add_library(WorldServer SHARED WorldServer.cc SmoresNode.cc SmoresEdge.cc SmoresModule.cc CollisionInformation.cc Condition.cc CommandManagement.cc)
target_link_libraries(WorldServer collision_message command_message ${GAZEBO_libraries} ${Boost_LIBRARIES})
add_dependencies(WorldServer collision_message command_message)