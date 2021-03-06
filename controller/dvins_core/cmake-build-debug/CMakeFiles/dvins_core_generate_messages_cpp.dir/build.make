# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/kevin/Apps/Clion/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/kevin/Apps/Clion/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kevin/UAV/dvins_core

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/UAV/dvins_core/cmake-build-debug

# Utility rule file for dvins_core_generate_messages_cpp.

# Include the progress variables for this target.
include CMakeFiles/dvins_core_generate_messages_cpp.dir/progress.make

CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/CommandArmServoAngleTrajectory.h
CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/PoseEuler.h
CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetectionInImageArray.h
CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetectionArray.h
CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/CommandArmServoAngle.h
CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetectionInImage.h
CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetection.h
CMakeFiles/dvins_core_generate_messages_cpp: devel/include/dvins_core/CommandMavAndGripper.h


devel/include/dvins_core/CommandArmServoAngleTrajectory.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/CommandArmServoAngleTrajectory.h: ../msg/CommandArmServoAngleTrajectory.msg
devel/include/dvins_core/CommandArmServoAngleTrajectory.h: ../msg/CommandArmServoAngle.msg
devel/include/dvins_core/CommandArmServoAngleTrajectory.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from dvins_core/CommandArmServoAngleTrajectory.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/CommandArmServoAngleTrajectory.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/dvins_core/PoseEuler.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/PoseEuler.h: ../msg/PoseEuler.msg
devel/include/dvins_core/PoseEuler.h: /opt/ros/melodic/share/std_msgs/msg/Header.msg
devel/include/dvins_core/PoseEuler.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from dvins_core/PoseEuler.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/PoseEuler.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/dvins_core/ArucoDetectionInImageArray.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/ArucoDetectionInImageArray.h: ../msg/ArucoDetectionInImageArray.msg
devel/include/dvins_core/ArucoDetectionInImageArray.h: ../msg/ArucoDetectionInImage.msg
devel/include/dvins_core/ArucoDetectionInImageArray.h: /opt/ros/melodic/share/std_msgs/msg/Header.msg
devel/include/dvins_core/ArucoDetectionInImageArray.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating C++ code from dvins_core/ArucoDetectionInImageArray.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/ArucoDetectionInImageArray.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/ArucoDetectionArray.h: ../msg/ArucoDetectionArray.msg
devel/include/dvins_core/ArucoDetectionArray.h: ../msg/ArucoDetection.msg
devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/share/geometry_msgs/msg/PoseWithCovarianceStamped.msg
devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/share/geometry_msgs/msg/Pose.msg
devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/share/geometry_msgs/msg/PoseWithCovariance.msg
devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/share/std_msgs/msg/Header.msg
devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/share/geometry_msgs/msg/Point.msg
devel/include/dvins_core/ArucoDetectionArray.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating C++ code from dvins_core/ArucoDetectionArray.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/ArucoDetectionArray.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/dvins_core/CommandArmServoAngle.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/CommandArmServoAngle.h: ../msg/CommandArmServoAngle.msg
devel/include/dvins_core/CommandArmServoAngle.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating C++ code from dvins_core/CommandArmServoAngle.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/CommandArmServoAngle.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/dvins_core/ArucoDetectionInImage.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/ArucoDetectionInImage.h: ../msg/ArucoDetectionInImage.msg
devel/include/dvins_core/ArucoDetectionInImage.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating C++ code from dvins_core/ArucoDetectionInImage.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/ArucoDetectionInImage.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/ArucoDetection.h: ../msg/ArucoDetection.msg
devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/share/geometry_msgs/msg/PoseWithCovarianceStamped.msg
devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/share/geometry_msgs/msg/Pose.msg
devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/share/geometry_msgs/msg/PoseWithCovariance.msg
devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/share/std_msgs/msg/Header.msg
devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/share/geometry_msgs/msg/Point.msg
devel/include/dvins_core/ArucoDetection.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating C++ code from dvins_core/ArucoDetection.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/ArucoDetection.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/lib/gencpp/gen_cpp.py
devel/include/dvins_core/CommandMavAndGripper.h: ../msg/CommandMavAndGripper.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/trajectory_msgs/msg/MultiDOFJointTrajectoryPoint.msg
devel/include/dvins_core/CommandMavAndGripper.h: ../msg/CommandArmServoAngle.msg
devel/include/dvins_core/CommandMavAndGripper.h: ../msg/CommandArmServoAngleTrajectory.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/geometry_msgs/msg/Twist.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/geometry_msgs/msg/Vector3.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/geometry_msgs/msg/Transform.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/std_msgs/msg/Header.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/trajectory_msgs/msg/MultiDOFJointTrajectory.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/geometry_msgs/msg/Quaternion.msg
devel/include/dvins_core/CommandMavAndGripper.h: /opt/ros/melodic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating C++ code from dvins_core/CommandMavAndGripper.msg"
	cd /home/kevin/UAV/dvins_core && /home/kevin/UAV/dvins_core/cmake-build-debug/catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/kevin/UAV/dvins_core/msg/CommandMavAndGripper.msg -Idvins_core:/home/kevin/UAV/dvins_core/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg -p dvins_core -o /home/kevin/UAV/dvins_core/cmake-build-debug/devel/include/dvins_core -e /opt/ros/melodic/share/gencpp/cmake/..

dvins_core_generate_messages_cpp: CMakeFiles/dvins_core_generate_messages_cpp
dvins_core_generate_messages_cpp: devel/include/dvins_core/CommandArmServoAngleTrajectory.h
dvins_core_generate_messages_cpp: devel/include/dvins_core/PoseEuler.h
dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetectionInImageArray.h
dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetectionArray.h
dvins_core_generate_messages_cpp: devel/include/dvins_core/CommandArmServoAngle.h
dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetectionInImage.h
dvins_core_generate_messages_cpp: devel/include/dvins_core/ArucoDetection.h
dvins_core_generate_messages_cpp: devel/include/dvins_core/CommandMavAndGripper.h
dvins_core_generate_messages_cpp: CMakeFiles/dvins_core_generate_messages_cpp.dir/build.make

.PHONY : dvins_core_generate_messages_cpp

# Rule to build all files generated by this target.
CMakeFiles/dvins_core_generate_messages_cpp.dir/build: dvins_core_generate_messages_cpp

.PHONY : CMakeFiles/dvins_core_generate_messages_cpp.dir/build

CMakeFiles/dvins_core_generate_messages_cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dvins_core_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dvins_core_generate_messages_cpp.dir/clean

CMakeFiles/dvins_core_generate_messages_cpp.dir/depend:
	cd /home/kevin/UAV/dvins_core/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/UAV/dvins_core /home/kevin/UAV/dvins_core /home/kevin/UAV/dvins_core/cmake-build-debug /home/kevin/UAV/dvins_core/cmake-build-debug /home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles/dvins_core_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dvins_core_generate_messages_cpp.dir/depend

