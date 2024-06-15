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

public:
	mpc_custom() {

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

	}

	~mpc_custom() {
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
    casadi::MX costFunction(const casadi::MX& x, const casadi::MX& desired_traj, int N, const casadi::MX& u) { //, const casadi::MX& u
        casadi::MX cost = 0.0;
        //double error_weight = ros::param::get("error_weight", error_weight);
        for (int i = 0; i < N; ++i) {
            std::cout << "iteration: " << i << std::endl;
            casadi::MX pose_error = x(casadi::Slice(), i) - desired_traj(casadi::Slice(),i);

            cost += pos_error_w * dot(pose_error, pose_error) + smooth_w * dot(u(casadi::Slice(), i), u(casadi::Slice(), i));
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

        casadi::MX desired_X = opti.variable(NUMBER_OF_STATES, N + 1); // desired state trajectory
        auto desired_x = desired_X(0,all);
        auto desired_y = desired_X(1,all);
        auto desired_z = desired_X(2,all);
        std::cout << "The variables have been set" << std::endl;

        casadi::MX U = opti.variable(NUMBER_OF_ACTUATIONS, N); // control trajectory (velocities)
        vel_x = U(0, all);
        vel_y = U(1, all);
        vel_z = U(2, all);

        // ---- objective ---------
        opti.minimize(costFunction(X(all, casadi::Slice(0, N)), desired_X(all, casadi::Slice(0,N)),N, U(all, casadi::Slice(0,N))));
        
        // ---- dynamic constraints --------
        for (int k = 0; k < N; ++k) {
            casadi::MX x_next = X(all, k) + dt * U(all, k);
            opti.subject_to(X(all,k+1) == x_next); // close the gaps

            // Force the desired poses to be the reference trajectory
            opti.subject_to(desired_x(k) == std::get<0>(Traj_ref[k]).pose.position.x);
            opti.subject_to(desired_y(k) == std::get<0>(Traj_ref[k]).pose.position.y);
            opti.subject_to(desired_z(k) == std::get<0>(Traj_ref[k]).pose.position.z);
        }

        // ---- path constraints -----------
        opti.subject_to(-VELOCITY_MAX <= U <= VELOCITY_MAX); // control limits (accelerations)

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

        // Evaluate and print the cost function and errors at each step
        double total_costfunct = 0.0;
        for (int k = 0; k < N; ++k) {
            double ref_x = std::get<0>(Traj_ref[k]).pose.position.x;
            double ref_y = std::get<0>(Traj_ref[k]).pose.position.y;
            double ref_z = std::get<0>(Traj_ref[k]).pose.position.z;

            double x_error = x_sol[k] - ref_x;
            double y_error = y_sol[k] - ref_y;
            double z_error = z_sol[k] - ref_z;
            double costfunct = pow(x_error,2) + pow(y_error,2) + pow(z_error,2);
            total_costfunct += costfunct;

            std::cout << "Step " << k << ": X error = " << x_error 
                        << ", Y error = " << y_error 
                        << ", Z error = " << z_error << " costfunction = " << costfunct << std::endl;
        }

        std::cout << "Total cost function = " << total_costfunct << std::endl; 

        std::vector<double> x_vel_sol = std::vector<double>(sol.value(vel_x));
        std::vector<double> y_vel_sol = std::vector<double>(sol.value(vel_y));
        std::vector<double> z_vel_sol = std::vector<double>(sol.value(vel_z));

        geometry_msgs::Twist vel_command; 

        vel_command.linear.x = x_vel_sol[0];
        vel_command.linear.y = y_vel_sol[0];
        vel_command.linear.z = z_vel_sol[0];
        velocity_pub_.publish(vel_command);
        ros::Duration(0.5).sleep(); 
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