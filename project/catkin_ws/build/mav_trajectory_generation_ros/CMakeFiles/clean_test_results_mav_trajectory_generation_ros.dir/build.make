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
CMAKE_SOURCE_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_trajectory_generation_ros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/mav_trajectory_generation_ros

# Utility rule file for clean_test_results_mav_trajectory_generation_ros.

# Include any custom commands dependencies for this target.
include CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/progress.make

CMakeFiles/clean_test_results_mav_trajectory_generation_ros:
	/usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/remove_test_results.py /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/mav_trajectory_generation_ros/test_results/mav_trajectory_generation_ros

clean_test_results_mav_trajectory_generation_ros: CMakeFiles/clean_test_results_mav_trajectory_generation_ros
clean_test_results_mav_trajectory_generation_ros: CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/build.make
.PHONY : clean_test_results_mav_trajectory_generation_ros

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/build: clean_test_results_mav_trajectory_generation_ros
.PHONY : CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/build

CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/clean

CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/depend:
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/mav_trajectory_generation_ros && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_trajectory_generation_ros /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/traj_gen/mav_trajectory_generation/mav_trajectory_generation_ros /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/mav_trajectory_generation_ros /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/mav_trajectory_generation_ros /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/mav_trajectory_generation_ros/CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/clean_test_results_mav_trajectory_generation_ros.dir/depend

