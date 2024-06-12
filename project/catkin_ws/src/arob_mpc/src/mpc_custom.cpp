#include "arob_mpc/mpc_custom.h"

using namespace std;

// CONSTRAINTS: State predefined bounds (px_k+1 = px_k - px_max), given state does not vary

// dx/dt = f(x,u)
casadi::MX f(const casadi::MX& x, const casadi::MX& u) {
  return vertcat(x(1), u(0), x(3), u(1), x(5), u(2));
}

int main() {
  int N = 100; // number of control intervals

  casadi::Opti opti = casadi::Opti(); // Optimization problem

  casadi::Slice all;
  // ---- decision variables ---------
  casadi::MX X = opti.variable(6, N + 1); // state trajectory [pos_x, vel_x, pos_y, vel_y, pos_z, vel_z]
  pos_x = X(0, all);
  vel_x = X(1, all);
  pos_y = X(2, all);
  vel_y = X(3, all);
  pos_z = X(4, all);
  vel_z = X(5, all);

  casadi::MX U = opti.variable(3, N); // control trajectory (accelerations)
  casadi::MX T = opti.variable(); // final time

  // ---- objective ---------
  
  // Define the cost function expression
  casadi::MX cost_function = opti.variable();

  // Tracking error to desired final position TODO: AQUÍ LA TRAYECTORIA
  double desired_pos_x = 0.0; // desired final position x
  double desired_pos_y = 2.0; // desired final position y
  double desired_pos_z = 2.0; // desired final position z

  for (int k = 0; k < N + 1; ++k) {
    cost_function += sumsqr(pos_x(k) - desired_pos_x) + sumsqr(pos_y(k) - desired_pos_y) + sumsqr(pos_z(k) - desired_pos_z);
  }

  // Control effort
  for (int k = 0; k < N; ++k) {
    control_effort += sumsqr(U(0, k)) + sumsqr(U(1, k)) + sumsqr(U(2, k));
  }

  // Smoothness of control inputs
  casadi::MX control_diff = 0;
  for (int k = 0; k < N-1; ++k) {
    control_diff += sumsqr(U(0, k+1) - U(0, k)) + sumsqr(U(1, k+1) - U(1, k)) + sumsqr(U(2, k+1) - U(2, k));
  }

  // Adjust weight factors as needed
  double w_tracking = 1.0;
  double w_control_effort = 0.1;
  double w_control_smoothness = 0.1;

  // Combine the cost terms with their respective weights
  opti.minimize(w_tracking * cost_function + w_control_effort * control_effort + w_control_smoothness * control_diff + T); 

  // ---- dynamic constraints --------
  casadi::MX dt = T / N;
  for (int k = 0; k < N; ++k) {
    casadi::MX k1 = f(X(all,k),         U(all,k));
    casadi::MX k2 = f(X(all,k)+dt/2*k1, U(all,k));
    casadi::MX k3 = f(X(all,k)+dt/2*k2, U(all,k));
    casadi::MX k4 = f(X(all,k)+dt*k3,   U(all,k));
    casadi::MX x_next = X(all,k) + dt/6*(k1+2*k2+2*k3+k4);
    opti.subject_to(X(all,k+1)==x_next); // close the gaps 
  }

  // ---- path constraints -----------
  opti.subject_to(-1 <= U <= 1); // control limits (accelerations) (backwards to forward)

  // ---- boundary conditions --------
  opti.subject_to(pos_x(0) == 0); // start position
  opti.subject_to(vel_x(0) == 0); // start with zero velocity
  opti.subject_to(pos_y(0) == 0);
  opti.subject_to(vel_y(0) == 0);
  opti.subject_to(pos_z(0) == 0);
  opti.subject_to(vel_z(0) == 0);

  // ---- Gates positions conditions --------
  opti.subject_to(pos_x(N) == 0); // end position
  opti.subject_to(pos_y(N) == 2);
  opti.subject_to(pos_z(N) == 2);

  // ---- misc. constraints ----------
  opti.subject_to(T >= 0); // Time must be positive

  // ---- initial values for solver ---
  opti.set_initial(T, 1); // Is it for computing the state at t+1?
  opti.set_initial(X, 0); // initial guess for state trajectory

  // ---- solve NLP ------
  opti.solver("ipopt"); // set numerical backend
  casadi::OptiSol sol = opti.solve(); // actual solve

  // Retrieve and print the results
  std::vector<double> x_sol = std::vector<double>(sol.value(pos_x));
  std::vector<double> y_sol = std::vector<double>(sol.value(pos_y));
  std::vector<double> z_sol = std::vector<double>(sol.value(pos_z));

  std::cout << "Optimal state trajectory:" << std::endl;
  for (size_t i = 0; i < x_sol.size(); ++i) {
    std::cout << "Step " << i << ": x = " << x_sol[i] << ", y = " << y_sol[i] << ", z = " << z_sol[i] << std::endl;
  }

  double T_sol = static_cast<double>(sol.value(T));
  std::cout << "Optimal final time: " << T_sol << std::endl;

  return 0;
}

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

    // geometry_msgs::PoseStamped current_pose; // Current pose of the quadrotor
    // geometry_msgs::Twist current_vel;
    // geometry_msgs::Accel current_accel;
    
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
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        Traj_ref.resize(N);

        // --- Optimization ---
        reset_all_states();

        initialize_bounds();

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

        // std::cout << "Printing received point :" << std::endl;

        // print_vector(msg.poses);

        // std::cout << "Printing overwritten vector :" << std::endl;

        // print_vector(Traj_goal);

        // Get the errors


        // Call the mpc method

        // Publish the new velocities obtained from the mpc


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


    // Update the pose based on the odometry
    // void accel_update(const nav_msgs::Odometry& msg){

        // // Get the accelerations        
        // std::get<2>(current_state).linear.x = msg.velocity.linear.x;
		// std::get<2>(current_state).linear.y = msg.velocity.linear.y;
		// std::get<2>(current_state).linear.y = msg.velocity.linear.z;

        // std::get<2>(current_state).angular.x = msg.velocity.angular.x;
        // std::get<2>(current_state).angular.y = msg.velocity.angular.y;
        // std::get<2>(current_state).angular.z = msg.velocity.angular.z;

    // }

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

    
    void mpc(){

        update_current_states();

        initialize_bounds();


        // TODO: Execute the solver

        geometry_msgs::Twist vel_command;
        vel_command.linear.x = solution.x[ID_FIRST_vx];
        vel_command.linear.y = solution.x[ID_FIRST_vy];
        vel_command.linear.z = solution.x[ID_FIRST_vz];

        vel_command.angular.x = solution.x[ID_FIRST_wx];
        vel_command.angular.y = solution.x[ID_FIRST_wy];
        vel_command.angular.z = solution.x[ID_FIRST_wz];

        ROS_INFO("HELOO2");

        // publish the velocities
        velocity_pub_.publish(vel_command);

    }

    // We initialize all states are set to zero
    void reset_all_states(){
        for (int i = 0; i < NX; ++i) {
            this->statesNactions[i] = 0.0;
        }
    }

    // We set the upper and lower limits of the variables and constraints
    void initialize_bounds(){

        //**************************************************************
        //* SET UPPER AND LOWER LIMITS OF VARIABLES
        //**************************************************************

        this->x_lowerbound.resize(NX);
        this->x_upperbound.resize(NX);

        // all other values large values the computer can handle
        // Postion + Orientation initialization
        for (int i = 0; i < ID_FIRST_vx; ++i) {
            this->x_lowerbound[i] = -1.0e10;
            this->x_upperbound[i] = 1.0e10;
        }

        // // Errors initialization
        // for (int i = ID_FIRST_p_error; i < ID_FIRST_ax; ++i) {
        //     this->x_lowerbound[i] = -1.0e10;
        //     this->x_upperbound[i] = 1.0e10;
        // }

        // all actuation inputs (velocities, accelerations) should have values between [-1, 1] or its maximum otherwise

        // // Velocities initialization
        // for (int i = ID_FIRST_vx; i < ID_FIRST_p_error; ++i) {
        //     this->x_lowerbound[i] = -VELOCITY_MAX;
        //     this->x_upperbound[i] = VELOCITY_MAX;
        // }

        // // Accelerations initialization
        // for (int i = ID_FIRST_ax; i < NX; ++i) {
        //     this->x_lowerbound[i] = -ACCEL_MAX;
        //     this->x_upperbound[i] = ACCEL_MAX;
        // }


        //**************************************************************
        //* SET UPPER AND LOWER LIMITS OF CONSTRAINTS
        //**************************************************************

        // the first constraint for each state variable
        // refer to the initial state conditions
        // this will be initialized when solve() is called
        // the succeeding constraints refer to the relationship
        // between succeeding states based on our kinematic model of the system

    }

    void update_current_states(){

        geometry_msgs::PoseStamped current_pos = std::get<0>(current_state);
        geometry_msgs::Twist current_vel  = std::get<1>(current_state);
        geometry_msgs::Accel current_accel = std::get<2>(current_state);

        std::vector<double> orientations = from_Quat_to_RPY(current_pos);

        const double px = current_pos.pose.position.x;
        const double py = current_pos.pose.position.y;
        const double pz = current_pos.pose.position.z;
        // const double roll = orientations[0];
        // const double pitch = orientations[1];
        // const double yaw = orientations[2];
        // const double vx = current_vel.linear.x;
        // const double vy = current_vel.linear.y;
        // const double vz = current_vel.linear.z;
        // const double wx = current_vel.angular.x;
        // const double wy = current_vel.angular.y;
        // const double wz = current_vel.angular.z;
        // const double ax = current_accel.linear.x;
        // const double ay = current_accel.linear.y;
        // const double az = current_accel.linear.z;
        // const double alpha_x = current_accel.angular.x;
        // const double alpha_y = current_accel.angular.y;
        // const double alpha_z = current_accel.angular.z;

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