cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(CEGUI CEGUI)
  pkg_check_modules(CEGUI-OGRE CEGUI-OGRE)
endif()

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS} ${CEGUI_INCLUDE_DIRS} ${CEGUI-OGRE_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/MessageDefinition)
# Include all folders in Util folder
FILE(GLOB _ALL_FILES ${CMAKE_CURRENT_BINARY_DIR}/../../../Util/ ${CMAKE_CURRENT_BINARY_DIR}/../../../Util/*)
FOREACH(_FILE ${_ALL_FILES})
  IF(IS_DIRECTORY ${_FILE})
    INCLUDE_DIRECTORIES(${_FILE})
  ENDIF()
ENDFOREACH()

link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS} ${CEGUI_LIBRARY_DIRS} ${CEGUI-OGRE_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/MessageDefinition)
add_subdirectory(../../MessageDefinition MessageDefinition)

add_library(system_gui SHARED system_gui.cc)
target_link_libraries(system_gui config_message ${GAZEBO_LIBRARIES} ${OGRE_LIBRARIES}
  ${OGRE-Terrain_LIBRARIES} ${CEGUI_LIBRARIES} ${CEGUI-OGRE_LIBRARIES})
add_dependencies(system_gui config_message)