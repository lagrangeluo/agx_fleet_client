cmake_minimum_required(VERSION 3.5.0)

project(agx_fleet_client_sim)
add_compile_options(-std=c++11)
#===============================================================================

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2 
    tf2_ros
)
if (catkin_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scout_messenger
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
  add_library(scout_messenger STATIC src/scout_messenger.cpp)
  target_link_libraries(scout_messenger ${catkin_LIBRARIES})
  add_dependencies(scout_messenger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_executable(scout_base_node src/scout_base_node.cpp)
  target_link_libraries(scout_base_node scout_messenger ${catkin_LIBRARIES})

  install(TARGETS scout_messenger scout_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  install(DIRECTORY include/scout_base/
  DESTINATION include/scout_base)

  install(
    DIRECTORY 
    launch
    maps 
    worlds
    param
    DESTINATION share/${PROJECT_NAME}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()
