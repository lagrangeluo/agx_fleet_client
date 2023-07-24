cmake_minimum_required(VERSION 3.5.0)

project(agx_fleet_client_sim)

#===============================================================================

find_package(catkin QUIET)

if (catkin_FOUND)

  catkin_package()

  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    DIRECTORY 
    maps 
    worlds
    param
    DESTINATION share/${PROJECT_NAME}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()
