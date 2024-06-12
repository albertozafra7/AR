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
#include <tf/transform_broadcaster.h>

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

using CppAD::AD;


const int N = 5; //10 How many steps we "lookahead" in the future we will use
const double dt = 0.1; // How much time we expect environment changes --> This has been configured to 0.1s in drone_race.cpp

//const double Lf = 2.67; // this is the length from front of vehicle to Center-of-Gravity
const double VELOCITY_MAX = 4.5; // this is what we ideally want our speed to always be --> also configured in drone_race.cpp
const double ACCEL_MAX = 8.65;

const int NUMBER_OF_STATES = 1;//22; // px, py, pz, roll, pitch, yaw, vx, vy, vz, wx, wy, wz, p_error, roll_error, pitch_error, yaw_error, ax, ay, az, alpha_x, alpha_y, alpha_z
const int NUMBER_OF_ACTUATIONS = 1;//6; // ax, ay, az, alpha_x, alpha_y, alpha_z


// weights for cost computations
const double W_p_error = 1500.0;
const double W_roll_error = 1500.0;
const double W_pitch_error = 1500.0;
const double W_yaw_error = 1500.0;
const double W_vx = 1.0;
const double W_vy = 1.0;
const double W_vz = 1.0;
const double W_wx = 1.0;
const double W_wy = 1.0;
const double W_wz = 1.0;
const double W_dvx = 1.0; // Weight costs for consecutives changes of velocity (In order to make it smooth)
const double W_dvy = 1.0;
const double W_dvz = 1.0;
const double W_dwx = 1.0;
const double W_dwy = 1.0;
const double W_dwz = 1.0;
const double W_ax = 1.0;
const double W_ay = 1.0;
const double W_az = 1.0;
const double W_alpha_x = 1.0;
const double W_alpha_y = 1.0;
const double W_alpha_z = 1.0;

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


// Type of the tuple used for storing the position, velocity and acceleration
typedef std::tuple<geometry_msgs::PoseStamped,geometry_msgs::Twist,geometry_msgs::Accel> quadrotor_data;

// class mpc_custom {

//  public:

//   double steer;
//   double throttle;

//   Dvector x; // where all the state and actuation variables will be stored
//   Dvector x_lowerbound; //lower limit for each corresponding variable in x
//   Dvector x_upperbound; //upper limit for each corresponding variable in x
//   Dvector g_lowerbound; // value constraint for each corresponding constraint expression
//   Dvector g_upperbound; // value constraint for each corresponding constraint expression

//   std::vector<double> future_xs;
//   std::vector<double> future_ys;

//   mpc_custom();
//   virtual ~mpc_custom();

//   // This callback receives the N poses computed by the trajectory generator
//   void goal_update(const arob_mpc::vector_poses& msg);

//   // This callback receives the pose update of the quadrotor
//   void position_update(const nav_msgs::Odometry& msg);


//   // this function solves the model given the current state and road curve coefficients.
//   void solve(Eigen::VectorXd state, Eigen::VectorXd K);
// };

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
