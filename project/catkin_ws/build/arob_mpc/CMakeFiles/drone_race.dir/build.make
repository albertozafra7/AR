# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/arob_mpc

# Include any dependencies generated for this target.
include CMakeFiles/drone_race.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/drone_race.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/drone_race.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/drone_race.dir/flags.make

CMakeFiles/drone_race.dir/src/drone_race.cpp.o: CMakeFiles/drone_race.dir/flags.make
CMakeFiles/drone_race.dir/src/drone_race.cpp.o: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/src/drone_race.cpp
CMakeFiles/drone_race.dir/src/drone_race.cpp.o: CMakeFiles/drone_race.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/arob_mpc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/drone_race.dir/src/drone_race.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/drone_race.dir/src/drone_race.cpp.o -MF CMakeFiles/drone_race.dir/src/drone_race.cpp.o.d -o CMakeFiles/drone_race.dir/src/drone_race.cpp.o -c /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/src/drone_race.cpp

CMakeFiles/drone_race.dir/src/drone_race.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/drone_race.dir/src/drone_race.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/src/drone_race.cpp > CMakeFiles/drone_race.dir/src/drone_race.cpp.i

CMakeFiles/drone_race.dir/src/drone_race.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/drone_race.dir/src/drone_race.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc/src/drone_race.cpp -o CMakeFiles/drone_race.dir/src/drone_race.cpp.s

# Object files for target drone_race
drone_race_OBJECTS = \
"CMakeFiles/drone_race.dir/src/drone_race.cpp.o"

# External object files for target drone_race
drone_race_EXTERNAL_OBJECTS =

/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: CMakeFiles/drone_race.dir/src/drone_race.cpp.o
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: CMakeFiles/drone_race.dir/build.make
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/mav_trajectory_generation_ros/lib/libmav_trajectory_generation_ros.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/mav_trajectory_generation/lib/libmav_trajectory_generation.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/glog_catkin/lib/libglog.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/nlopt/lib/libnlopt_wrap.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/mav_visualization/lib/libmav_visualization.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/libeigen_conversions.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/liborocos-kdl.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/libroscpp.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/librosconsole.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/librostime.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/libcpp_common.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/eigen_checks/lib/libeigen_checks.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/libroslib.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /opt/ros/noetic/lib/librospack.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race: CMakeFiles/drone_race.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/arob_mpc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drone_race.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/drone_race.dir/build: /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/arob_mpc/lib/arob_mpc/drone_race
.PHONY : CMakeFiles/drone_race.dir/build

CMakeFiles/drone_race.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/drone_race.dir/cmake_clean.cmake
.PHONY : CMakeFiles/drone_race.dir/clean

CMakeFiles/drone_race.dir/depend:
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/arob_mpc && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/arob_mpc /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/arob_mpc /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/arob_mpc /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/arob_mpc/CMakeFiles/drone_race.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/drone_race.dir/depend

