#include "arob_mpc/mpc_custom.h"

using namespace std;

// +++++++++++++++++++++++++++ MPC (ROS MANAGEMENT) +++++++++++++++++++++++++++

class mpc_custom {
	ros::NodeHandle nh_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
    ros::Subscriber velocity_sub_;
    // ros::Subscriber accel_sub_;
    ros::Subscriber state_sub_;
	ros::Subscriber goal_sub_;

	std::vector<quadrotor_data> Traj_ref;  // Line to follow
    
    
    // This is done in order to compute the acceleration by an estimation of the increment/decrement of the measured speed
    std_msgs::Header prev_header;
    
    // This tuple contains the current 
    quadrotor_data current_state;

    // File path for storing the trajectory
    string file_path;
    std::ofstream file;
    int global_cont = 0;
    // Evaluate and print the cost function and errors at each step
    double total_costfunct = 0.0;


public:
	mpc_custom() {

        loadParams();

        prev_header.stamp.sec = 0;
        prev_header.stamp.nsec = 0;

        // --- ROS ---
		// Subscribe to the /base_pose_groun_truth topic
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &mpc_custom::pose_update, this);
        // // Subscribe to the / topic
        velocity_sub_ = nh_.subscribe("cmd_vel", 1, &mpc_custom::velocity_update, this);
        // // Subscribe to the / topic
        // accel_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &mpc_custom::accel_update, this);
        // Subscribe to the /ground_truth/state topic
        state_sub_ = nh_.subscribe("/ground_truth/state", 1, &mpc_custom::state_update, this);
		// Subscribe to the /goal_pos topic that give us the trajectory to follow with a lookahead
		goal_sub_ = nh_.subscribe("goal_pos", 1, &mpc_custom::goal_update, this);
		// Sets as publisher to the velocity topic
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

        Traj_ref.resize(N);

        // Open file to save poses
        file.open(file_path);

	}

	~mpc_custom() {
        file << "Total trajectory error = " << std::sqrt(total_costfunct) << std::endl;
        file << "Mean step error = " << std::sqrt(total_costfunct)/global_cont << std::endl;
        // Close file
        file.close();
	}



    void goal_update(const arob_mpc::vector_poses& msg){

        ROS_INFO("The Custom MPC has received a new trajectory, with a lookahead of %ld to follow", msg.poses.size());

        for(size_t i = 0; i < msg.poses.size(); ++i){
            std::get<0>(Traj_ref[i]) = msg.poses[i];
            std::get<1>(Traj_ref[i]) = msg.velocities[i];
            std::get<2>(Traj_ref[i]) = msg.accelerations[i];
        }

        mpc();
    }


    void loadParams() {
        // yaml params loader
        nh_.getParam("drone_params/lookahead", N);
        nh_.getParam("drone_params/sampling_interval", dt);
        nh_.getParam("drone_params/max_vel", VELOCITY_MAX);
        nh_.getParam("drone_params/max_accel", ACCEL_MAX);
        nh_.getParam("drone_params/pos_error_w", pos_error_w);
        smooth_w = 1 - pos_error_w - orient_error_w;

        nh_.getParam("drone_params/saving_file", file_path);
    }

    // Update the pose based on the odometry
    void pose_update(const nav_msgs::Odometry& msg){

        // Get the positions        
        std::get<0>(current_state).pose.position.x = msg.pose.pose.position.x;
		std::get<0>(current_state).pose.position.y = msg.pose.pose.position.y;
		std::get<0>(current_state).pose.position.z = msg.pose.pose.position.z;

        std::get<0>(current_state).pose.orientation.x = msg.pose.pose.orientation.x;
        std::get<0>(current_state).pose.orientation.y = msg.pose.pose.orientation.y;
        std::get<0>(current_state).pose.orientation.z = msg.pose.pose.orientation.z;
        std::get<0>(current_state).pose.orientation.w = msg.pose.pose.orientation.w;

    }


    // Update the velocity based on the Twist
    void velocity_update(const geometry_msgs::Twist& msg){

        // Get the velocities        
        std::get<1>(current_state).linear.x = msg.linear.x;
		std::get<1>(current_state).linear.y = msg.linear.y;
		std::get<1>(current_state).linear.y = msg.linear.z;

        std::get<1>(current_state).angular.x = msg.angular.x;
        std::get<1>(current_state).angular.y = msg.angular.y;
        std::get<1>(current_state).angular.z = msg.angular.z;

    }

    // Update all states based on the odometry
    void state_update(const nav_msgs::Odometry& msg){


        // Get the positions        
        std::get<0>(current_state).pose.position.x = msg.pose.pose.position.x;
		std::get<0>(current_state).pose.position.y = msg.pose.pose.position.y;
		std::get<0>(current_state).pose.position.z = msg.pose.pose.position.z;

        std::get<0>(current_state).pose.orientation.x = msg.pose.pose.orientation.x;
        std::get<0>(current_state).pose.orientation.y = msg.pose.pose.orientation.y;
        std::get<0>(current_state).pose.orientation.z = msg.pose.pose.orientation.z;
        std::get<0>(current_state).pose.orientation.w = msg.pose.pose.orientation.w;

        // We compute the acceleration
        std_msgs::Header current_header = msg.header;
        float d_time = (current_header.stamp.sec + current_header.stamp.nsec/1e9) - (prev_header.stamp.sec + prev_header.stamp.nsec/1e9);
        std::get<2>(current_state).linear.x = (msg.twist.twist.linear.x - std::get<1>(current_state).linear.x)/(d_time);
		std::get<2>(current_state).linear.y = (msg.twist.twist.linear.y - std::get<1>(current_state).linear.y)/(d_time);
		std::get<2>(current_state).linear.y = (msg.twist.twist.linear.z - std::get<1>(current_state).linear.z)/(d_time);
        
        std::get<2>(current_state).angular.x = (msg.twist.twist.angular.x - std::get<1>(current_state).angular.x)/(d_time);
        std::get<2>(current_state).angular.y = (msg.twist.twist.angular.y - std::get<1>(current_state).angular.y)/(d_time);
        std::get<2>(current_state).angular.z = (msg.twist.twist.angular.z - std::get<1>(current_state).angular.z)/(d_time);

        // We update the header
        prev_header = current_header;


        // Get the velocities        
        std::get<1>(current_state).linear.x = msg.twist.twist.linear.x;
		std::get<1>(current_state).linear.y = msg.twist.twist.linear.y;
		std::get<1>(current_state).linear.y = msg.twist.twist.linear.z;

        std::get<1>(current_state).angular.x = msg.twist.twist.angular.x;
        std::get<1>(current_state).angular.y = msg.twist.twist.angular.y;
        std::get<1>(current_state).angular.z = msg.twist.twist.angular.z;

    } 


    // Cost function
    casadi::MX costFunction(const casadi::MX& x, const casadi::MX& desired_traj, int N, const casadi::MX& u) {
        casadi::MX cost = 0.0;
        for (int i = 0; i < N; ++i) {
            std::cout << "iteration: " << i << std::endl;
            casadi::MX pose_error = x(casadi::Slice(), i) - desired_traj(casadi::Slice(),i);

            // casadi::MX pose_error = x(casadi::Slice(0, 3), i) - desired_traj(casadi::Slice(0, 3),i);
            // casadi::MX orientation_error = x(casadi::Slice(3, 6), i) - desired_traj(casadi::Slice(3, 6),i);

            cost += pos_error_w * dot(pose_error, pose_error) + smooth_w * dot(u(casadi::Slice(), i), u(casadi::Slice(), i));
            //cost += pos_error_w * dot(pose_error, pose_error) + orient_error_w * dot(orientation_error, orientation_error) + smooth_w * dot(u(casadi::Slice(), i), u(casadi::Slice(), i));
        }
        return cost;
    }

    
    void mpc(){
        ROS_INFO("The MPC is executing");

        casadi::Opti opti = casadi::Opti(); // Optimization problem

        casadi::Slice all;
        // ---- decision variables ---------
        casadi::MX X = opti.variable(NUMBER_OF_STATES, N + 1); // state trajectory [pos_x, pos_y, pos_z]
        pos_x = X(0, all);
        pos_y = X(1, all);
        pos_z = X(2, all);
        // roll = X(3, all);
        // pitch = X(4, all);
        // yaw = X(5, all);

        casadi::MX desired_X = opti.variable(NUMBER_OF_STATES, N + 1); // desired state trajectory
        auto desired_x = desired_X(0,all);
        auto desired_y = desired_X(1,all);
        auto desired_z = desired_X(2,all);
        // auto desired_roll = desired_X(3, all);
        // auto desired_pitch = desired_X(4, all);
        // auto desired_yaw = desired_X(5, all);
        std::cout << "The variables have been set" << std::endl;

        casadi::MX U = opti.variable(NUMBER_OF_ACTUATIONS, N); // control trajectory (velocities)
        vel_x = U(0, all);
        vel_y = U(1, all);
        vel_z = U(2, all);
        // roll = U(3, all);
        // pitch = U(4, all);
        // yaw = U(5, all);

        // ---- objective ---------
        opti.minimize(costFunction(X(all, casadi::Slice(0, N)), desired_X(all, casadi::Slice(0,N)),N, U(all, casadi::Slice(0,N))));
        
        // ---- dynamic constraints --------
        for (int k = 0; k < N; ++k) {
            casadi::MX x_next = X(all, k) + dt * U(all, k);
            opti.subject_to(X(all,k+1) == x_next); // close the gaps
            // casadi::MX x_next = X(casadi::Slice(0, 3), k) + dt * U(casadi::Slice(0, 3), k);
            // casadi::MX orientation_next = X(casadi::Slice(3, 6), k) + dt * U(casadi::Slice(3, 6), k);

            // opti.subject_to(X(casadi::Slice(0, 3), k+1) == x_next);
            // opti.subject_to(X(casadi::Slice(3, 6), k+1) == orientation_next);

            // Force the desired poses to be the reference trajectory
            opti.subject_to(desired_x(k) == std::get<0>(Traj_ref[k]).pose.position.x);
            opti.subject_to(desired_y(k) == std::get<0>(Traj_ref[k]).pose.position.y);
            opti.subject_to(desired_z(k) == std::get<0>(Traj_ref[k]).pose.position.z);

            // Calculate desired orientation based on the direction of motion
            // casadi::MX desired_roll_k = 0; // Assuming roll is zero in this example, you can change it as needed
            // casadi::MX desired_pitch_k = atan2(U(2, k), sqrt(pow(U(0, k), 2) + pow(U(1, k), 2))); // pitch based on velocity
            // casadi::MX desired_yaw_k = atan2(U(1, k), U(0, k)); // yaw based on velocity

            // opti.subject_to(desired_roll(k) == desired_roll_k);
            // opti.subject_to(desired_pitch(k) == desired_pitch_k);
            // opti.subject_to(desired_yaw(k) == desired_yaw_k);
        }

        // ---- path constraints -----------
        opti.subject_to(-VELOCITY_MAX <= U <= VELOCITY_MAX); // control limits (accelerations)
        // opti.subject_to(-VELOCITY_MAX <= U(casadi::Slice(0, 3), all) <= VELOCITY_MAX); // control limits (velocities)
        // opti.subject_to(-ANGULAR_RATE_MAX <= U(casadi::Slice(3, 6), all) <= ANGULAR_RATE_MAX); // control limits (angular rates)    

        ROS_INFO("Current State:");
        std::cout << "Poses : (" << std::get<0>(current_state).pose.position.x << "," << std::get<0>(current_state).pose.position.y << "," << std::get<0>(current_state).pose.position.z << ")" << std::endl;
        std::cout << "Vels : (" << std::get<1>(current_state).linear.x << "," << std::get<1>(current_state).linear.y << "," << std::get<1>(current_state).linear.z << ")" << std::endl;

        ROS_INFO("Next_Pose:");
        std::cout << "Poses : (" << std::get<0>(Traj_ref[0]).pose.position.x << "," << std::get<0>(Traj_ref[0]).pose.position.y << "," << std::get<0>(Traj_ref[0]).pose.position.z << ")" << std::endl;
        std::cout << "Vels : (" << std::get<1>(Traj_ref[0]).linear.x << "," << std::get<1>(Traj_ref[0]).linear.y << "," << std::get<1>(Traj_ref[0]).linear.z << ")" << std::endl;

        // ---- boundary conditions --------
        opti.subject_to(pos_x(0) == std::get<0>(current_state).pose.position.x); // start position
        opti.subject_to(pos_y(0) == std::get<0>(current_state).pose.position.y);
        opti.subject_to(pos_z(0) == std::get<0>(current_state).pose.position.z);
        // opti.subject_to(roll(0) == std::get<0>(current_state).pose.orientation.x); // start roll
        // opti.subject_to(pitch(0) == std::get<0>(current_state).pose.orientation.y); // start pitch
        // opti.subject_to(yaw(0) == std::get<0>(current_state).pose.orientation.z); // start yaw


        // ---- initial values for solver ---
        opti.set_initial(X, 0); // initial guess for state trajectory
        opti.set_initial(U, 0); // initial guess for control trajectory

        // ---- solve NLP ------
        casadi::Dict opts_dict=casadi::Dict();
        opts_dict["ipopt.print_level"] = 0;
        opts_dict["ipopt.sb"] = "yes";
        opts_dict["print_time"] = 0;
        opti.solver("ipopt", opts_dict); // set numerical backend
        casadi::OptiSol sol = opti.solve(); // actual solve

        // Retrieve and print the results
        std::vector<double> x_sol = std::vector<double>(sol.value(pos_x));
        std::vector<double> y_sol = std::vector<double>(sol.value(pos_y));
        std::vector<double> z_sol = std::vector<double>(sol.value(pos_z));
        // std::vector<double> roll_sol = std::vector<double>(sol.value(roll));
        // std::vector<double> pitch_sol = std::vector<double>(sol.value(pitch));
        // std::vector<double> yaw_sol = std::vector<double>(sol.value(yaw));

        

        if (file.is_open()) {
            double ref_x = std::get<0>(Traj_ref[0]).pose.position.x;
            double ref_y = std::get<0>(Traj_ref[0]).pose.position.y;
            double ref_z = std::get<0>(Traj_ref[0]).pose.position.z;
            // double ref_roll = 0; // Assuming desired roll is zero
            // double ref_pitch = atan2(std::get<1>(Traj_ref[k]).linear.z, sqrt(pow(std::get<1>(Traj_ref[k]).linear.x, 2) + pow(std::get<1>(Traj_ref[k]).linear.y, 2)));
            // double ref_yaw = atan2(std::get<1>(Traj_ref[k]).linear.y, std::get<1>(Traj_ref[k]).linear.x);

            double x_error = std::get<0>(current_state).pose.position.x - ref_x;
            double y_error = std::get<0>(current_state).pose.position.y - ref_y;
            double z_error = std::get<0>(current_state).pose.position.z - ref_z;
            // double roll_error = roll_sol[k] - ref_roll;
            // double pitch_error = pitch_sol[k] - ref_pitch;
            // double yaw_error = yaw_sol[k] - ref_yaw;
            double costfunct = pow(x_error, 2) + pow(y_error, 2) + pow(z_error, 2);
            // double costfunct = pow(x_error, 2) + pow(y_error, 2) + pow(z_error, 2) + pow(roll_error, 2) + pow(pitch_error, 2) + pow(yaw_error, 2);
            total_costfunct += costfunct;

            // Write to the file
            file << "Step " << global_cont << "; ";
            file << "X error = " << x_error << ", Y error = " << y_error << ", Z error = " << z_error << ", ";
            // file << "Roll error = " << roll_errorr << ", Pitch error = " << pitch_error << ", Yaw error = " << yaw_error << ", ";
            file << "Step Global Error = " << std::sqrt(costfunct) << "\n";

            global_cont++;

        } else {
            ROS_ERROR("Unable to open file for writing positions.");
        }

        std::vector<double> x_vel_sol = std::vector<double>(sol.value(vel_x));
        std::vector<double> y_vel_sol = std::vector<double>(sol.value(vel_y));
        std::vector<double> z_vel_sol = std::vector<double>(sol.value(vel_z));
        
        // std::vector<double> roll_vel_sol = std::vector<double>(sol.value(roll));
        // std::vector<double> pitch_vel_sol = std::vector<double>(sol.value(pitch));
        // std::vector<double> yaw_vel_sol = std::vector<double>(sol.value(yaw));

        geometry_msgs::Twist vel_command; 

        vel_command.linear.x = x_vel_sol[0];
        vel_command.linear.y = y_vel_sol[0];
        vel_command.linear.z = z_vel_sol[0];
        // vel_command.angular.x = roll_vel_sol[0];
        // vel_command.angular.y = pitch_vel_sol[0];
        // vel_command.angular.z = yaw_vel_sol[0];
        velocity_pub_.publish(vel_command);
        ros::Duration(dt).sleep(); 
    }


    void update_current_states(){

        geometry_msgs::PoseStamped current_pos = std::get<0>(current_state);
        geometry_msgs::Twist current_vel  = std::get<1>(current_state);
        geometry_msgs::Accel current_accel = std::get<2>(current_state);

        std::vector<double> orientations = from_Quat_to_RPY(current_pos);

        const double px = current_pos.pose.position.x;
        const double py = current_pos.pose.position.y;
        const double pz = current_pos.pose.position.z;

    }

    // We update the current upper and lower bounds for the constraints
    // The assignation is with its current state value for ensuring a smooth movement
    void update_current_constraint_bounds(){

        
    }
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "mpc_custom");
	ros::NodeHandle nh("~");
	mpc_custom mpc;

	ros::spin();
	return 0;
}