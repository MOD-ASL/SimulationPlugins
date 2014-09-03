cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(octomap_DIR "./octomap/lib/cmake/octomap/")

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(CEGUI CEGUI)
  pkg_check_modules(CEGUI-OGRE CEGUI-OGRE)
endif()

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS} ${CEGUI_INCLUDE_DIRS} ${CEGUI-OGRE_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS} ${CEGUI_LIBRARY_DIRS} ${CEGUI-OGRE_LIBRARY_DIRS})

add_library(painting_plugin SHARED PaintingPlugin.cc)
target_link_libraries(painting_plugin ${GAZEBO_LIBRARIES} ${OGRE_LIBRARIES}
  ${OGRE-Terrain_LIBRARIES} ${CEGUI_LIBRARIES} ${CEGUI-OGRE_LIBRARIES})
