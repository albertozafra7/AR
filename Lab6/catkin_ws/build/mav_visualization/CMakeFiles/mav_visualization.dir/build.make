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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization

# Include any dependencies generated for this target.
include CMakeFiles/mav_visualization.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mav_visualization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mav_visualization.dir/flags.make

CMakeFiles/mav_visualization.dir/src/marker_group.cpp.o: CMakeFiles/mav_visualization.dir/flags.make
CMakeFiles/mav_visualization.dir/src/marker_group.cpp.o: /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/marker_group.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mav_visualization.dir/src/marker_group.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mav_visualization.dir/src/marker_group.cpp.o -c /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/marker_group.cpp

CMakeFiles/mav_visualization.dir/src/marker_group.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mav_visualization.dir/src/marker_group.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/marker_group.cpp > CMakeFiles/mav_visualization.dir/src/marker_group.cpp.i

CMakeFiles/mav_visualization.dir/src/marker_group.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mav_visualization.dir/src/marker_group.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/marker_group.cpp -o CMakeFiles/mav_visualization.dir/src/marker_group.cpp.s

CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.o: CMakeFiles/mav_visualization.dir/flags.make
CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.o: /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/leica_marker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.o -c /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/leica_marker.cpp

CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/leica_marker.cpp > CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.i

CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/leica_marker.cpp -o CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.s

CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.o: CMakeFiles/mav_visualization.dir/flags.make
CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.o: /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/hexacopter_marker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.o -c /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/hexacopter_marker.cpp

CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/hexacopter_marker.cpp > CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.i

CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization/src/hexacopter_marker.cpp -o CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.s

# Object files for target mav_visualization
mav_visualization_OBJECTS = \
"CMakeFiles/mav_visualization.dir/src/marker_group.cpp.o" \
"CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.o" \
"CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.o"

# External object files for target mav_visualization
mav_visualization_EXTERNAL_OBJECTS =

/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: CMakeFiles/mav_visualization.dir/src/marker_group.cpp.o
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: CMakeFiles/mav_visualization.dir/src/leica_marker.cpp.o
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: CMakeFiles/mav_visualization.dir/src/hexacopter_marker.cpp.o
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: CMakeFiles/mav_visualization.dir/build.make
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/libeigen_conversions.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/liborocos-kdl.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/libroscpp.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/librosconsole.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/librostime.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /opt/ros/noetic/lib/libcpp_common.so
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so: CMakeFiles/mav_visualization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mav_visualization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mav_visualization.dir/build: /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so

.PHONY : CMakeFiles/mav_visualization.dir/build

CMakeFiles/mav_visualization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mav_visualization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mav_visualization.dir/clean

CMakeFiles/mav_visualization.dir/depend:
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_visualization /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization /home/albertozafra7/Desktop/Universidad/Master/AR/Lab6/catkin_ws/build/mav_visualization/CMakeFiles/mav_visualization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mav_visualization.dir/depend

