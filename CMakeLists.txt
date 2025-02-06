
# Author: Michele Sprocatti, Lorenzo Serafini, Zlatko Kovachev
cmake_minimum_required(VERSION 3.0.2)
project(ir2425_group_07_assignment2)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    tiago_iaslab_simulation
    message_generation
    actionlib
    apriltag_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    tf
    moveit_ros_planning_interface
    moveit_core
    moveit_ros_planning
    gazebo_ros_link_attacher
)

add_service_files(
  FILES
  waypoint_service.srv
  collision_add_service.srv
  collision_rem_service.srv
  srvArm.srv
  point_line.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib moveit_ros_planning_interface message_runtime
)

include_directories(
    include
    include/ir2425_group_07_assignment2
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_WAYPOINT_GENERATOR
  include/ir2425_group_07_assignment2/waypoint_generator.h
  src/waypoint_generator.cpp
)

add_library(${PROJECT_NAME}_COLLISION_DETECTOR
  include/ir2425_group_07_assignment2/collision_detector.h
  src/collision_detector.cpp
)

add_library(${PROJECT_NAME}_UTILITIES
  include/ir2425_group_07_assignment2/utilities.h
  src/utilities.cpp
)

add_library(${PROJECT_NAME}_PICKNPLACE
  include/ir2425_group_07_assignment2/pick_n_place.h
  src/pick_n_place.cpp
)

add_executable(${PROJECT_NAME}_nodeA src/nodeA.cpp)
add_executable(${PROJECT_NAME}_waypoint_navigation src/waypoint_navigation.cpp)
add_executable(${PROJECT_NAME}_detection src/detection.cpp)
add_executable(${PROJECT_NAME}_pointLine src/point_line_server.cpp)
add_executable(${PROJECT_NAME}_arm src/arm_move.cpp)

add_dependencies(${PROJECT_NAME}_nodeA ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_waypoint_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_UTILITIES ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_PICKNPLACE ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_COLLISION_DETECTOR ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_pointLine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_waypoint_navigation ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_pointLine ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_arm ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_COLLISION_DETECTOR
  ${PROJECT_NAME}_UTILITIES
  ${catkin_LIBRARIES} 
)
target_link_libraries(${PROJECT_NAME}_UTILITIES
  ${catkin_LIBRARIES} 
)
target_link_libraries(${PROJECT_NAME}_PICKNPLACE
  ${catkin_LIBRARIES} 
)
target_link_libraries(${PROJECT_NAME}_nodeA 
  ${PROJECT_NAME}_UTILITIES 
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_detection 
  ${PROJECT_NAME}_COLLISION_DETECTOR
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_waypoint_navigation 
  ${PROJECT_NAME}_WAYPOINT_GENERATOR
  ${catkin_LIBRARIES}  
)
target_link_libraries(${PROJECT_NAME}_arm 
  ${PROJECT_NAME}_PICKNPLACE
  ${PROJECT_NAME}_UTILITIES
  ${catkin_LIBRARIES}  
)

