cmake_minimum_required(VERSION 3.0.2)
project(docking_marker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  move_base_msgs
  message_generation
)

################################################
## Message generation
################################################
add_message_files(
  FILES
  DockTarget.msg
)

add_message_files(
  FILES
  DockingStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Catkin package
################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    move_base_msgs
    message_runtime
)

################################################
## Include directories
################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################
## Executables
################################################
add_executable(reflector_cluster_detector src/reflector_cluster_detector.cpp)
add_dependencies(reflector_cluster_detector docking_marker_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(reflector_cluster_detector ${catkin_LIBRARIES})

add_executable(reflector_cluster_detector_constraint src/reflector_cluster_detector_constraint.cpp)
add_dependencies(reflector_cluster_detector_constraint docking_marker_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(reflector_cluster_detector_constraint ${catkin_LIBRARIES})

add_executable(reflector_cluster_detector_dual src/reflector_cluster_detector_dual.cpp)
add_dependencies(reflector_cluster_detector_dual docking_marker_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(reflector_cluster_detector_dual ${catkin_LIBRARIES})

add_executable(docking_controller src/docking_controller.cpp)
add_dependencies(docking_controller docking_marker_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_controller ${catkin_LIBRARIES})

add_executable(docking_controller_dual src/docking_controller_dual.cpp)
add_dependencies(docking_controller_dual docking_marker_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_controller_dual ${catkin_LIBRARIES})

add_executable(dual_reflector_detector src/dual_reflector_cluster_detector.cpp)
target_link_libraries(dual_reflector_detector ${catkin_LIBRARIES})

add_executable(dual_reflector_docking_controller src/dual_reflector_docking_controller.cpp)
target_link_libraries(dual_reflector_docking_controller ${catkin_LIBRARIES})


add_executable(mission_controller src/mission_controller.cpp)
target_link_libraries(mission_controller ${catkin_LIBRARIES})
add_dependencies(mission_controller ${catkin_EXPORTED_TARGETS})

add_executable(mission_controller_retry_dock src/mission_controller_retry_dock.cpp)
target_link_libraries(mission_controller_retry_dock ${catkin_LIBRARIES})
add_dependencies(mission_controller_retry_dock ${catkin_EXPORTED_TARGETS})


add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
target_link_libraries(initial_pose_publisher ${catkin_LIBRARIES})
add_dependencies(initial_pose_publisher ${catkin_EXPORTED_TARGETS})

add_executable(amcl_trail_publisher src/amcl_trail_publisher.cpp)
target_link_libraries(amcl_trail_publisher ${catkin_LIBRARIES})
add_dependencies(amcl_trail_publisher ${catkin_EXPORTED_TARGETS})

