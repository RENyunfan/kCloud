# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dvins_core: 8 messages, 0 services")

set(MSG_I_FLAGS "-Idvins_core:/home/kevin/UAV/dvins_core/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dvins_core_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg" "dvins_core/CommandArmServoAngle"
)

get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg" "dvins_core/ArucoDetectionInImage:std_msgs/Header"
)

get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg" "dvins_core/ArucoDetection:geometry_msgs/PoseWithCovarianceStamped:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg" ""
)

get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg" ""
)

get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg" "geometry_msgs/PoseWithCovarianceStamped:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg" NAME_WE)
add_custom_target(_dvins_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dvins_core" "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg" "trajectory_msgs/MultiDOFJointTrajectoryPoint:dvins_core/CommandArmServoAngle:dvins_core/CommandArmServoAngleTrajectory:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Transform:std_msgs/Header:trajectory_msgs/MultiDOFJointTrajectory:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)
_generate_msg_cpp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
)

### Generating Services

### Generating Module File
_generate_module_cpp(dvins_core
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dvins_core_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dvins_core_generate_messages dvins_core_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_cpp _dvins_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvins_core_gencpp)
add_dependencies(dvins_core_gencpp dvins_core_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvins_core_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)
_generate_msg_eus(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
)

### Generating Services

### Generating Module File
_generate_module_eus(dvins_core
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dvins_core_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dvins_core_generate_messages dvins_core_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_eus _dvins_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvins_core_geneus)
add_dependencies(dvins_core_geneus dvins_core_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvins_core_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)
_generate_msg_lisp(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
)

### Generating Services

### Generating Module File
_generate_module_lisp(dvins_core
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dvins_core_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dvins_core_generate_messages dvins_core_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_lisp _dvins_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvins_core_genlisp)
add_dependencies(dvins_core_genlisp dvins_core_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvins_core_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)
_generate_msg_nodejs(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dvins_core
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dvins_core_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dvins_core_generate_messages dvins_core_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_nodejs _dvins_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvins_core_gennodejs)
add_dependencies(dvins_core_gennodejs dvins_core_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvins_core_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)
_generate_msg_py(dvins_core
  "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg;/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
)

### Generating Services

### Generating Module File
_generate_module_py(dvins_core
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dvins_core_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dvins_core_generate_messages dvins_core_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/PoseEuler.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/ArucoDetection.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg" NAME_WE)
add_dependencies(dvins_core_generate_messages_py _dvins_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dvins_core_genpy)
add_dependencies(dvins_core_genpy dvins_core_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dvins_core_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dvins_core
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dvins_core_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(dvins_core_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dvins_core
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dvins_core_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(dvins_core_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dvins_core
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dvins_core_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(dvins_core_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dvins_core
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dvins_core_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(dvins_core_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dvins_core
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dvins_core_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(dvins_core_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
