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
CMAKE_SOURCE_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/unique_identifier/unique_id

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id

# Utility rule file for clean_test_results_unique_id.

# Include the progress variables for this target.
include tests/CMakeFiles/clean_test_results_unique_id.dir/progress.make

tests/CMakeFiles/clean_test_results_unique_id:
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id/tests && /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/remove_test_results.py /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id/test_results/unique_id

clean_test_results_unique_id: tests/CMakeFiles/clean_test_results_unique_id
clean_test_results_unique_id: tests/CMakeFiles/clean_test_results_unique_id.dir/build.make

.PHONY : clean_test_results_unique_id

# Rule to build all files generated by this target.
tests/CMakeFiles/clean_test_results_unique_id.dir/build: clean_test_results_unique_id

.PHONY : tests/CMakeFiles/clean_test_results_unique_id.dir/build

tests/CMakeFiles/clean_test_results_unique_id.dir/clean:
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id/tests && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_unique_id.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/clean_test_results_unique_id.dir/clean

tests/CMakeFiles/clean_test_results_unique_id.dir/depend:
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/unique_identifier/unique_id /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/unique_identifier/unique_id/tests /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id/tests /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/unique_id/tests/CMakeFiles/clean_test_results_unique_id.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/clean_test_results_unique_id.dir/depend

