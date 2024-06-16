#ifndef mpc_custom_H
#define mpc_custom_H


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
// #include <tf/transform_broadcaster.h>

#include <iostream>
#include <sstream>
#include <stdio.h> 
#include <math.h>
#include <fstream>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include <casadi/casadi.hpp>

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <ros/package.h>

#include "arob_mpc/vector_poses.h"


int N; //10 How many steps we "lookahead" in the future we will use
double dt; // How much time we expect environment changes --> This has been configured to 0.1s in drone_race.cpp
 
double VELOCITY_MAX; // this is what we ideally want our speed to always be --> also configured in drone_race.cpp
double ACCEL_MAX;

const int NUMBER_OF_STATES = 3; // px, py, pz, roll, pitch, yaw, vx, vy, vz, wx, wy, wz, p_error, roll_error, pitch_error, yaw_error, ax, ay, az, alpha_x, alpha_y, alpha_z
const int NUMBER_OF_ACTUATIONS = 3; // ax, ay, az, alpha_x, alpha_y, alpha_z

double pos_error_w;
double smooth_w;
double orient_error_w;

// Variables to store the values to optimize
casadi::MX pos_x;
casadi::MX pos_y;
casadi::MX pos_z;
casadi::MX roll;
casadi::MX pitch;
casadi::MX yaw;
casadi::MX vel_x;
casadi::MX vel_y;
casadi::MX vel_z;
casadi::MX wx;
casadi::MX wy;
casadi::MX wz;
casadi::MX accel_x;
casadi::MX accel_y;
casadi::MX accel_z;
casadi::MX alpha_x;
casadi::MX alpha_y;
casadi::MX alpha_z;
// TODO: revisar


// Type of the tuple used for storing the position, velocity and acceleration
typedef std::tuple<geometry_msgs::PoseStamped,geometry_msgs::Twist,geometry_msgs::Accel> quadrotor_data;

std::vector<double> from_Quat_to_RPY(const geometry_msgs::PoseStamped& pose_stamped){
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose_stamped.pose.orientation, quaternion);

    // Convert quaternion to roll, pitch, and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    std::vector<double> rpy;
    rpy.push_back(roll);
    rpy.push_back(pitch);
    rpy.push_back(yaw);

    return rpy;
}

#endif /* mpc_custom_h */
