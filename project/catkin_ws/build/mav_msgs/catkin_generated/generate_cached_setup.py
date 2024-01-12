# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel;/home/albertozafra7/tb2_ws/devel_isolated/mltbot_pr1;/home/albertozafra7/catkin_ws/devel;/home/albertozafra7/tb2_ws/devel_isolated/yujin_ocs;/home/albertozafra7/tb2_ws/devel_isolated/yocs_waypoints_navi;/home/albertozafra7/tb2_ws/devel_isolated/yocs_waypoint_provider;/home/albertozafra7/tb2_ws/devel_isolated/yocs_virtual_sensor;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_qtestsuite;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_testsuite;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_node;/home/albertozafra7/tb2_ws/devel_isolated/yocs_velocity_smoother;/home/albertozafra7/tb2_ws/devel_isolated/yocs_safety_controller;/home/albertozafra7/tb2_ws/devel_isolated/yocs_rapps;/home/albertozafra7/tb2_ws/devel_isolated/yocs_navigator;/home/albertozafra7/tb2_ws/devel_isolated/yocs_navi_toolkit;/home/albertozafra7/tb2_ws/devel_isolated/yocs_joyop;/home/albertozafra7/tb2_ws/devel_isolated/yocs_ar_pair_tracking;/home/albertozafra7/tb2_ws/devel_isolated/yocs_msgs;/home/albertozafra7/tb2_ws/devel_isolated/yocs_diff_drive_pose_controller;/home/albertozafra7/tb2_ws/devel_isolated/yocs_ar_marker_tracking;/home/albertozafra7/tb2_ws/devel_isolated/yocs_math_toolkit;/home/albertozafra7/tb2_ws/devel_isolated/yocs_localization_manager;/home/albertozafra7/tb2_ws/devel_isolated/yocs_keyop;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_safety_controller;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_random_walker;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_controller_tutorial;/home/albertozafra7/tb2_ws/devel_isolated/yocs_controllers;/home/albertozafra7/tb2_ws/devel_isolated/yocs_cmd_vel_mux;/home/albertozafra7/tb2_ws/devel_isolated/yocs_ar_pair_approach;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_teleop;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_stdr;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_stage;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_simulator;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_rviz_launchers;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_rapps;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_navigation;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_follower;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_msgs;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_interactive_markers;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_interactions;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_gazebo;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_description;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_dashboard;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_capabilities;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_calibration;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_bringup;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_apps;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot_actions;/home/albertozafra7/tb2_ws/devel_isolated/turtlebot;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_driver;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_auto_docking;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_dock_drive;/home/albertozafra7/tb2_ws/devel_isolated/ecl_statistics;/home/albertozafra7/tb2_ws/devel_isolated/ecl_mobile_robot;/home/albertozafra7/tb2_ws/devel_isolated/ecl_core_apps;/home/albertozafra7/tb2_ws/devel_isolated/ecl_geometry;/home/albertozafra7/tb2_ws/devel_isolated/ecl_linear_algebra;/home/albertozafra7/tb2_ws/devel_isolated/slam_gmapping;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_rviz_launchers;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_rapps;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_keyop;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_gazebo_plugins;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_dashboard;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_bumper2pc;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_msgs;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_gazebo;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_ftdi;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_desktop;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_description;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_core;/home/albertozafra7/tb2_ws/devel_isolated/kobuki_capabilities;/home/albertozafra7/tb2_ws/devel_isolated/kobuki;/home/albertozafra7/tb2_ws/devel_isolated/gmapping;/home/albertozafra7/tb2_ws/devel_isolated/ecl_streams;/home/albertozafra7/tb2_ws/devel_isolated/ecl_sigslots;/home/albertozafra7/tb2_ws/devel_isolated/ecl_devices;/home/albertozafra7/tb2_ws/devel_isolated/ecl_threads;/home/albertozafra7/tb2_ws/devel_isolated/ecl_containers;/home/albertozafra7/tb2_ws/devel_isolated/ecl_utilities;/home/albertozafra7/tb2_ws/devel_isolated/ecl_math;/home/albertozafra7/tb2_ws/devel_isolated/ecl_formatters;/home/albertozafra7/tb2_ws/devel_isolated/ecl_converters;/home/albertozafra7/tb2_ws/devel_isolated/ecl_concepts;/home/albertozafra7/tb2_ws/devel_isolated/ecl_type_traits;/home/albertozafra7/tb2_ws/devel_isolated/ecl_tools;/home/albertozafra7/tb2_ws/devel_isolated/ecl_ipc;/home/albertozafra7/tb2_ws/devel_isolated/ecl_time;/home/albertozafra7/tb2_ws/devel_isolated/ecl_time_lite;/home/albertozafra7/tb2_ws/devel_isolated/ecl_sigslots_lite;/home/albertozafra7/tb2_ws/devel_isolated/ecl_navigation;/home/albertozafra7/tb2_ws/devel_isolated/ecl_mpl;/home/albertozafra7/tb2_ws/devel_isolated/ecl_lite;/home/albertozafra7/tb2_ws/devel_isolated/ecl_io;/home/albertozafra7/tb2_ws/devel_isolated/ecl_filesystem;/home/albertozafra7/tb2_ws/devel_isolated/ecl_exceptions;/home/albertozafra7/tb2_ws/devel_isolated/ecl_errors;/home/albertozafra7/tb2_ws/devel_isolated/ecl_eigen;/home/albertozafra7/tb2_ws/devel_isolated/ecl_converters_lite;/home/albertozafra7/tb2_ws/devel_isolated/ecl_console;/home/albertozafra7/tb2_ws/devel_isolated/ecl_config;/home/albertozafra7/tb2_ws/devel_isolated/ecl_command_line;/home/albertozafra7/tb2_ws/devel_isolated/ecl_build;/home/albertozafra7/tb2_ws/devel_isolated/ecl_license;/home/albertozafra7/tb2_ws/devel_isolated/ecl_core;/home/albertozafra7/tb2_ws/devel_isolated/depthimage_to_laserscan;/home/albertozafra7/tb2_ws/devel_isolated/ddynamic_reconfigure;/home/albertozafra7/tb2_ws/devel_isolated/ar_track_alvar_msgs;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/devel/.private/mav_msgs/env.sh')

output_filename = '/home/albertozafra7/Desktop/Universidad/Master/AR/project/catkin_ws/build/mav_msgs/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
