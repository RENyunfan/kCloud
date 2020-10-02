cmake_minimum_required(VERSION 2.4.6)
project(dvins_core)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  cmake_modules
  image_geometry
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  mavros_msgs
  tf2
  tf2_ros
  trajectory_msgs
  # rotors_gazebo
  # apriltag_ros
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_message_files(
  FILES
  ArucoDetectionInImage.msg
  ArucoDetectionInImageArray.msg
  ArucoDetection.msg
  ArucoDetectionArray.msg
  CommandArmServoAngle.msg
  PoseEuler.msg
  CommandArmServoAngleTrajectory.msg
  CommandMavAndGripper.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
)


catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  image_geometry
  geometry_msgs
  roscpp
  rospy
  mav_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen_LIBRARIES}
)

add_executable(aruco_detector src/aruco_detector.cpp)
target_link_libraries(aruco_detector ${catkin_LIBRARIES})

# add_executable(switch_node src/switch_node.cpp)
# target_link_libraries(switch_node ${catkin_LIBRARIES})

# add_executable(sw1 src/sw1.cpp)
# target_link_libraries(sw1 ${catkin_LIBRARIES})

# add_executable(dvins_estimator src/adap_estimator.cpp)
# target_link_libraries(dvins_estimator ${catkin_LIBRARIES})

add_executable(dvins_estimator src/dvins_estimator.cpp)
target_link_libraries(dvins_estimator ${catkin_LIBRARIES})

add_executable(dvins_estimator1 src/dvins_estimator1.cpp)
target_link_libraries(dvins_estimator1 ${catkin_LIBRARIES})

add_executable(dvins_estimator2 src/dvins_estimator2.cpp)
target_link_libraries(dvins_estimator2 ${catkin_LIBRARIES})

add_executable(hovering_example src/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})

add_executable(hover_loop src/hover_loop.cpp)
target_link_libraries(hover_loop ${catkin_LIBRARIES})

add_library(gazebo_cable_plugin src/gazebo_cable_plugin.cpp)
target_link_libraries(gazebo_cable_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_cable_plugin ${catkin_EXPORTED_TARGETS})

add_library(lee_payload_position_controller src/lee_payload_position_controller.cpp)
target_link_libraries(lee_payload_position_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(lee_payload_position_controller ${catkin_EXPORTED_TARGETS})

add_library(arm_plugin src/arm_plugin.cpp)
target_link_libraries(arm_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(arm_plugin ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_publisher_rectangle src/trajectory_publisher_rectangle.cpp)
target_link_libraries(trajectory_publisher_rectangle ${catkin_LIBRARIES})
add_dependencies(trajectory_publisher_rectangle ${catkin_EXPORTED_TARGETS})


install(
  TARGETS
  arm_plugin gazebo_cable_plugin lee_payload_position_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(lee_payload src/lee_payload_position_controller_node.cpp)
target_link_libraries(lee_payload lee_payload_position_controller ${catkin_LIBRARIES})

add_library(lpc
  src/lpc.cpp
)

target_link_libraries(lpc ${catkin_LIBRARIES})
add_dependencies(lpc ${catkin_EXPORTED_TARGETS})

add_executable(lpcn src/lpcn.cpp)
add_dependencies(lpcn ${catkin_EXPORTED_TARGETS})
target_link_libraries(lpcn
  lpc ${catkin_LIBRARIES})

add_library(lee_position_controller
  src/lee_position_controller.cpp
)

target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
add_dependencies(lee_position_controller ${catkin_EXPORTED_TARGETS})

add_executable(lee_position_controller_node src/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node
  lee_position_controller ${catkin_LIBRARIES})
