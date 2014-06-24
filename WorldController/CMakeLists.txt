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
	${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/
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
# add_subdirectory(../../WorldPlugin WorldPlugin)

add_library(WorldController SHARED WorldController.cc 
		${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/WorldServer.cc 
		${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/SmoresNode.cc 
		${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/SmoresEdge.cc 
		${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/SmoresModule.cc 
		${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/CollisionInformation.cc 
		${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/Condition.cc 
		${CMAKE_CURRENT_BINARY_DIR}/../../WorldPlugin/CommandManagement.cc)
target_link_libraries(WorldController collision_message command_message config_message ${GAZEBO_libraries} ${Boost_LIBRARIES})
add_dependencies(WorldController collision_message command_message config_message)