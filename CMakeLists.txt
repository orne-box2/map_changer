cmake_minimum_required(VERSION 3.0.2)
project(map_changer)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    waypoint_manager_msgs
    nav_msgs
    geometry_msgs
    tf
    std_msgs
    map_server
)

find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_changer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(map_changer_node src/map_changer_node.cpp)
add_library(map_changer_config src/config.cpp)
target_compile_features(map_changer_config PUBLIC cxx_std_17)
target_link_libraries(map_changer_config yaml-cpp ${catkin_LIBRARIES})
add_dependencies(map_changer_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(map_changer_node map_changer_config ${catkin_LIBRARIES})

install(TARGETS map_changer_node map_changer_config
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config map waypoint_config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
