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

# Include any dependencies generated for this target.
include CMakeFiles/hovering_example.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hovering_example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hovering_example.dir/flags.make

CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o: CMakeFiles/hovering_example.dir/flags.make
CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o: ../src/hovering_example.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o -c /home/kevin/UAV/dvins_core/src/hovering_example.cpp

CMakeFiles/hovering_example.dir/src/hovering_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hovering_example.dir/src/hovering_example.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/UAV/dvins_core/src/hovering_example.cpp > CMakeFiles/hovering_example.dir/src/hovering_example.cpp.i

CMakeFiles/hovering_example.dir/src/hovering_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hovering_example.dir/src/hovering_example.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/UAV/dvins_core/src/hovering_example.cpp -o CMakeFiles/hovering_example.dir/src/hovering_example.cpp.s

# Object files for target hovering_example
hovering_example_OBJECTS = \
"CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o"

# External object files for target hovering_example
hovering_example_EXTERNAL_OBJECTS =

devel/lib/dvins_core/hovering_example: CMakeFiles/hovering_example.dir/src/hovering_example.cpp.o
devel/lib/dvins_core/hovering_example: CMakeFiles/hovering_example.dir/build.make
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libimage_geometry.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libtf2_ros.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libactionlib.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libroscpp.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/librosconsole.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libtf2.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/librostime.so
devel/lib/dvins_core/hovering_example: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/dvins_core/hovering_example: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/dvins_core/hovering_example: CMakeFiles/hovering_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/dvins_core/hovering_example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hovering_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hovering_example.dir/build: devel/lib/dvins_core/hovering_example

.PHONY : CMakeFiles/hovering_example.dir/build

CMakeFiles/hovering_example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hovering_example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hovering_example.dir/clean

CMakeFiles/hovering_example.dir/depend:
	cd /home/kevin/UAV/dvins_core/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/UAV/dvins_core /home/kevin/UAV/dvins_core /home/kevin/UAV/dvins_core/cmake-build-debug /home/kevin/UAV/dvins_core/cmake-build-debug /home/kevin/UAV/dvins_core/cmake-build-debug/CMakeFiles/hovering_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hovering_example.dir/depend
