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
CMAKE_SOURCE_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/traj_gen/nlopt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt

# Utility rule file for nlopt_src.

# Include any custom commands dependencies for this target.
include CMakeFiles/nlopt_src.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/nlopt_src.dir/progress.make

CMakeFiles/nlopt_src: CMakeFiles/nlopt_src-complete

CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-install
CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-mkdir
CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-download
CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-update
CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-patch
CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-configure
CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-build
CMakeFiles/nlopt_src-complete: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-install
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'nlopt_src'"
	/home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E make_directory /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles
	/home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles/nlopt_src-complete
	/home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-done

nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-build: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing build step for 'nlopt_src'"
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-build && make
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-build && /home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-build

nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-configure: nlopt_src-prefix/tmp/nlopt_src-cfgcmd.txt
nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-configure: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Performing configure step for 'nlopt_src'"
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-build && nlopt-2.4.2/configure --with-cxx --without-matlab --with-pic --prefix=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/install
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-build && /home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-configure

nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-download: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-custominfo.txt
nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-download: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step for 'nlopt_src'"
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src && wget https://github.com/ethz-asl/thirdparty_library_binaries/raw/master/nlopt-2.4.2.tar.gz
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src && /home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-download

nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-install: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-build
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing install step for 'nlopt_src'"
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-build && /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/make_install_nlopt.sh
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-build && /home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-install

nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'nlopt_src'"
	/home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -Dcfgdir= -P /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/tmp/nlopt_src-mkdirs.cmake
	/home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-mkdir

nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-patch: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-patch-info.txt
nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-patch: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-update
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing patch step for 'nlopt_src'"
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src && tar -xzf ../nlopt-2.4.2.tar.gz && rm -rf ../nlopt_src-build/nlopt-2.4.2 && mv nlopt-2.4.2 ../nlopt_src-build/
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src && /home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-patch

nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-update: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-update-info.txt
nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-update: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-download
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No update step for 'nlopt_src'"
	/home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E echo_append
	/home/albertozafra7/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E touch /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-update

nlopt_src: CMakeFiles/nlopt_src
nlopt_src: CMakeFiles/nlopt_src-complete
nlopt_src: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-build
nlopt_src: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-configure
nlopt_src: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-download
nlopt_src: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-install
nlopt_src: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-mkdir
nlopt_src: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-patch
nlopt_src: nlopt_src-prefix/src/nlopt_src-stamp/nlopt_src-update
nlopt_src: CMakeFiles/nlopt_src.dir/build.make
.PHONY : nlopt_src

# Rule to build all files generated by this target.
CMakeFiles/nlopt_src.dir/build: nlopt_src
.PHONY : CMakeFiles/nlopt_src.dir/build

CMakeFiles/nlopt_src.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/nlopt_src.dir/cmake_clean.cmake
.PHONY : CMakeFiles/nlopt_src.dir/clean

CMakeFiles/nlopt_src.dir/depend:
	cd /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/traj_gen/nlopt /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/src/traj_gen/nlopt /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt /home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/nlopt/CMakeFiles/nlopt_src.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/nlopt_src.dir/depend

