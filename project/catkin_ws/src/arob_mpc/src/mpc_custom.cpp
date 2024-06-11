#include "arob_mpc/mpc_custom.h"

using namespace std;

// CONSTRAINTS: State predefined bounds (px_k+1 = px_k - px_max), given state does not vary

// If libpopt gives an error run first this in the terminal
// export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

// +++++++++++++++++++++++++++ MPC (IPOPT OPTIMIZATION) +++++++++++++++++++++++++++
class FG_eval {

public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    // Fitted polynomial coefficients
    std::vector<quadrotor_data> Traj_ref;  // Line to follow

    FG_eval(std::vector<quadrotor_data> Traj_in) : Traj_ref(Traj_in) {}

    void operator()(ADvector& fg, const ADvector& statesNactions) {


        fg[0] =  CppAD::pow(statesNactions[ID_FIRST_px] - 3, 2); //cost_function(statesNactions);

        std::cout << "Coste = " << statesNactions[ID_FIRST_px]  << std::endl;

        ROS_INFO("The cost function has been computed");

        constraints_statement(fg,statesNactions);

        ROS_INFO("The constraints are correctly created");
    }

    // We compute the costfunction J(sk,uk,duk) that we have to optimize with mpc
    AD<double> cost_function(const ADvector& statesNactions){
        ROS_INFO("Calculating the costfunction");

        // Remember J(sk,uk,duk) = for(i=1;i<N)(w_error * error²) + for(j=0;j < N-1)(w_action * action_increment²) 
        //                         + for(k=0;k < N-2)(w_action_derivate * action_derivate_increment²) --> This is required for smooth actions

        // Remember that N is the lookahead and we are computing errors in position and orientation

        AD<double> cost_funct = 0.0;
        for(int i = 0; i < N; ++i){

            //------------------
            //----- ERRORS -----
            //------------------

            // Error in position and in Orientation
            const AD<double> p_error = statesNactions[ID_FIRST_p_error + i];
            const AD<double> roll_error = statesNactions[ID_FIRST_roll_error + i];
            const AD<double> pitch_error = statesNactions[ID_FIRST_pitch_error + i];
            const AD<double> yaw_error = statesNactions[ID_FIRST_yaw_error + i];

            //            _______ position error ______ + _____________ roll error _____________ + ______________ pitch error ______________
            cost_funct += W_p_error * p_error * p_error + W_roll_error * roll_error * roll_error + W_pitch_error * pitch_error * pitch_error
                        + W_yaw_error * yaw_error * yaw_error;  // yaw error


            //-------------------
            //----- Actions -----
            //-------------------
            if(i < N-1){
                // Increment of linear and angular velocities --> vx, vy, vz, wx, wy, wz
                const AD<double> delta_vx = statesNactions[ID_FIRST_vx + i];
                const AD<double> delta_vy = statesNactions[ID_FIRST_vy + i];
                const AD<double> delta_vz = statesNactions[ID_FIRST_vz + i];
                const AD<double> delta_wx = statesNactions[ID_FIRST_wx + i];
                const AD<double> delta_wy = statesNactions[ID_FIRST_wy + i];
                const AD<double> delta_wz = statesNactions[ID_FIRST_wz + i];

                //           __________________________________ linear velocity __________________________________ 
                cost_funct += W_vx * delta_vx * delta_vx + W_vy * delta_vy * delta_vy + W_vz * delta_vz * delta_vz
                //            + _________________________________ angular velocity _________________________________       
                              + W_wx * delta_wx * delta_wx + W_wy * delta_wy * delta_wy + W_wz * delta_wz * delta_wz;
       
            }


            //-----------------------------
            //----- Smoothing Actions -----
            //-----------------------------
            if(i < N-2){

                // Increment of linear and angular velocities in the next timestep --> vx, vy, vz, wx, wy, wz
                const AD<double> ddelta_vx = statesNactions[ID_FIRST_vx + i + 1] - statesNactions[ID_FIRST_vx + i];
                const AD<double> ddelta_vy = statesNactions[ID_FIRST_vy + i + 1] - statesNactions[ID_FIRST_vy + i];
                const AD<double> ddelta_vz = statesNactions[ID_FIRST_vz + i + 1] - statesNactions[ID_FIRST_vz + i];
                const AD<double> ddelta_wx = statesNactions[ID_FIRST_wx + i + 1] - statesNactions[ID_FIRST_wx + i];
                const AD<double> ddelta_wy = statesNactions[ID_FIRST_wy + i + 1] - statesNactions[ID_FIRST_wy + i];
                const AD<double> ddelta_wz = statesNactions[ID_FIRST_wz + i + 1] - statesNactions[ID_FIRST_wz + i];
                //           __________________________________ linear velocity __________________________________ 
                cost_funct += W_dvx * ddelta_vx * ddelta_vx + W_dvy * ddelta_vy * ddelta_vy + W_dvz * ddelta_vz * ddelta_vz
                //            + _________________________________ angular velocity _________________________________       
                              + W_dwx * ddelta_wx * ddelta_wx + W_dwy * ddelta_wy * ddelta_wy + W_dwz * ddelta_wz * ddelta_wz;
            }


        }
        return cost_funct;
    }


    // We predict the future states based on the system model
    //void states_prediction(ADvector& statesNactions){
        // Remember that the system model (dynamics) is the following:
            // px_k+1 = px_k + timestep * vx_k
            // py_k+1 = py_k + timestep * vy_k
            // pz_k+1 = pz_k + timestep * vz_k
            // roll_k+1 = roll_k + timestep * wx_k
            // pitch_k+1 = pitch_k + timestep * wy_k
            // yaw_k+1 = yaw_k + timestep * wz_k

            // Same for velocities and angular velocities
            // vx_k+1 = vx_k + timestep * ax_k
            // wx_k+1 = wx_k + timestep * ax_ang_k

            // More constraints
            // v < vmax
            // a < amax, will be calculated as the increment of velocities/time 


    //}

    // We stablish the constraints of the mpc model
    void constraints_statement(ADvector& fg, const ADvector& statesNactions){
        // This is done to ensure that the first position is the current position
        fg[ID_FIRST_px + 1] = statesNactions[ID_FIRST_px];
        // fg[ID_FIRST_py + 1] = statesNactions[ID_FIRST_py];
        // fg[ID_FIRST_pz + 1] = statesNactions[ID_FIRST_pz];
        // fg[ID_FIRST_roll + 1] = statesNactions[ID_FIRST_roll];
        // fg[ID_FIRST_pitch + 1] = statesNactions[ID_FIRST_pitch];
        // fg[ID_FIRST_yaw + 1] = statesNactions[ID_FIRST_yaw];
        fg[ID_FIRST_vx + 1] = statesNactions[ID_FIRST_vx];
        // fg[ID_FIRST_vy + 1] = statesNactions[ID_FIRST_vy];
        // fg[ID_FIRST_vz + 1] = statesNactions[ID_FIRST_vz];
        // fg[ID_FIRST_wx + 1] = statesNactions[ID_FIRST_wx];
        // fg[ID_FIRST_wy + 1] = statesNactions[ID_FIRST_wy];
        // fg[ID_FIRST_wz + 1] = statesNactions[ID_FIRST_wz];
        // fg[ID_FIRST_ax + 1] = statesNactions[ID_FIRST_ax];
        // fg[ID_FIRST_ay + 1] = statesNactions[ID_FIRST_ay];
        // fg[ID_FIRST_az + 1] = statesNactions[ID_FIRST_az];
        // fg[ID_FIRST_alpha_x + 1] = statesNactions[ID_FIRST_alpha_x];
        // fg[ID_FIRST_alpha_y + 1] = statesNactions[ID_FIRST_alpha_y];
        // fg[ID_FIRST_alpha_z + 1] = statesNactions[ID_FIRST_alpha_z];

        // constraints based on our kinematic model
        for (int i = 0; i < N - 1; ++i) {

            // where the current state variables of interest are stored
            // stored for readability
            const int ID_CURRENT_px = ID_FIRST_px + i;
            // const int ID_CURRENT_py = ID_FIRST_py + i;
            // const int ID_CURRENT_pz = ID_FIRST_pz + i;
            // const int ID_CURRENT_roll = ID_FIRST_roll + i;
            // const int ID_CURRENT_pitch = ID_FIRST_roll + i;
            // const int ID_CURRENT_yaw = ID_FIRST_pitch + i;
            const int ID_CURRENT_vx = ID_FIRST_vx + i;
            // const int ID_CURRENT_vy = ID_FIRST_vy + i;
            // const int ID_CURRENT_vz = ID_FIRST_vz + i;
            // const int ID_CURRENT_wx = ID_FIRST_wx + i;
            // const int ID_CURRENT_wy = ID_FIRST_wy + i;
            // const int ID_CURRENT_wz = ID_FIRST_wz + i;
            // const int ID_CURRENT_ax = ID_FIRST_ax + i;
            // const int ID_CURRENT_ay = ID_FIRST_ay + i;
            // const int ID_CURRENT_az = ID_FIRST_az + i;
            // const int ID_CURRENT_alpha_x = ID_FIRST_alpha_x + i;
            // const int ID_CURRENT_alpha_y = ID_FIRST_alpha_y + i;
            // const int ID_CURRENT_alpha_z = ID_FIRST_alpha_z + i;

            // current state and actuations
            const auto px0 = statesNactions[ID_CURRENT_px];
            // const auto py0 = statesNactions[ID_CURRENT_py];
            // const auto pz0 = statesNactions[ID_CURRENT_pz];
            // const auto roll0 = statesNactions[ID_CURRENT_roll];
            // const auto pitch0 = statesNactions[ID_CURRENT_pitch];
            // const auto yaw0 = statesNactions[ID_CURRENT_yaw];
            const auto vx0 = statesNactions[ID_CURRENT_vx];
            // const auto vy0 = statesNactions[ID_CURRENT_vy];
            // const auto vz0 = statesNactions[ID_CURRENT_vz];
            // const auto wx0 = statesNactions[ID_CURRENT_wx];
            // const auto wy0 = statesNactions[ID_CURRENT_wy];
            // const auto wz0 = statesNactions[ID_CURRENT_wz];
            // const auto ax0 = statesNactions[ID_CURRENT_ax];
            // const auto ay0 = statesNactions[ID_CURRENT_ay];
            // const auto az0 = statesNactions[ID_CURRENT_az];
            // const auto alpha_x0 = statesNactions[ID_CURRENT_alpha_x];
            // const auto alpha_y0 = statesNactions[ID_CURRENT_alpha_y];
            // const auto alpha_z0 = statesNactions[ID_CURRENT_alpha_z];

            // next states and actuations
            const auto px1 = statesNactions[ID_CURRENT_px + 1];
            // const auto py1 = statesNactions[ID_CURRENT_py + 1];
            // const auto pz1 = statesNactions[ID_CURRENT_pz + 1];
            // const auto roll1 = statesNactions[ID_CURRENT_roll + 1];
            // const auto pitch1 = statesNactions[ID_CURRENT_pitch + 1];
            // const auto yaw1 = statesNactions[ID_CURRENT_yaw + 1];
            // const auto vx1 = statesNactions[ID_CURRENT_vx + 1];
            // const auto vy1 = statesNactions[ID_CURRENT_vy + 1];
            // const auto vz1 = statesNactions[ID_CURRENT_vz + 1];
            // const auto wx1 = statesNactions[ID_CURRENT_wx + 1];
            // const auto wy1 = statesNactions[ID_CURRENT_wy + 1];
            // const auto wz1 = statesNactions[ID_CURRENT_wz + 1];
            // const auto ax1 = statesNactions[ID_CURRENT_ax + 1];
            // const auto ay1 = statesNactions[ID_CURRENT_ay + 1];
            // const auto az1 = statesNactions[ID_CURRENT_az + 1];
            // const auto alpha_x1 = statesNactions[ID_CURRENT_alpha_x + 1];
            // const auto alpha_y1 = statesNactions[ID_CURRENT_alpha_y + 1];
            // const auto alpha_z1 = statesNactions[ID_CURRENT_alpha_z + 1];
            
            // relationship of current state + actuations and next state
            // based on our kinematic model
            const auto px1_predict = px0 + vx0 * dt;
            // const auto py1_predict = py0 + vy0 * dt;
            // const auto pz1_predict = py0 + vz0 * dt;
            // const auto roll1_predict = roll0 + wx0 * dt;
            // const auto pitch1_predict = pitch0 + wy0 * dt;
            // const auto yaw1_predict = yaw0 + wz0 * dt;
            // const auto vx1_predict = vx0 + ax0 * dt;
            // const auto vy1_predict = vy0 + ay0 * dt;
            // const auto vz1_predict = vz0 + az0 * dt;
            // const auto wx1_predict = wx0 + alpha_x0 * dt;
            // const auto wy1_predict = wy0 + alpha_y0 * dt;
            // const auto wz1_predict = wz0 + alpha_z0 * dt;

            // store the constraint expression of two consecutive states
            fg[ID_CURRENT_px + 2] = px1 - px1_predict;
            // fg[ID_CURRENT_py + 2] = py1 - py1_predict;
            // fg[ID_CURRENT_pz + 2] = pz1 - pz1_predict;
            // fg[ID_CURRENT_roll + 2] = roll1 - roll1_predict;
            // fg[ID_CURRENT_pitch + 2] = pitch1 - pitch1_predict;
            // fg[ID_CURRENT_yaw + 2] = yaw1 - yaw1_predict;
            // fg[ID_CURRENT_vx + 2] = vx1 - vx1_predict;
            // fg[ID_CURRENT_vy + 2] = vy1 - vy1_predict;
            // fg[ID_CURRENT_vz + 2] = vz1 - vz1_predict;
            // fg[ID_CURRENT_wx + 2] = wx1 - wx1_predict;
            // fg[ID_CURRENT_wy + 2] = wy1 - wy1_predict;
            // fg[ID_CURRENT_wz + 2] = wz1 - wz1_predict;

            // If we add v < vmax we have to add a new step
            // AD<double> v1_predict = sqrt(vx1_predict*vx1_predict + vy1_predict*vy1_predict + vz1_predict*vz1_predict);
            // AD<double> w1_predict = sqrt(wx1_predict*wx1_predict + wy1_predict*wy1_predict + wz1_predict*wz1_predict);
            // fg[ID_CURRENT_ax + 2] = v1_predict; // we overwrite the acceleration for the memory shake
            // fg[ID_CURRENT_ay + 2] = w1_predict;
            // fg[ID_CURRENT_az + 2] = (v1_predict - sqrt(vx0*vx0 + vy0*vy0 + vz0*vz0))/dt; // accel
            // fg[ID_CURRENT_alpha_x + 2] = (w1_predict - sqrt(wx0*wx0 + wy0*wy0 + wz0*wz0))/dt;
        }

    }


    AD<double> euclidean_distance(const AD<double> origin_x, const AD<double> origin_y, const AD<double> origin_z, geometry_msgs::Point dest){
        return euclidean_distance(ADToPoint(origin_x,origin_y,origin_z),dest);
    }


    AD<double> euclidean_distance(geometry_msgs::Point origin, geometry_msgs::Point dest){
        return sqrt(pow(dest.x - origin.x,2) + pow(dest.y - origin.y,2) + pow(dest.z - origin.z,2));
    }

    geometry_msgs::Point ADToPoint(const AD<double> x, const AD<double> y, const AD<double> z){
        geometry_msgs::Point position;
        position.x = CppAD::Value(x);
        position.y = CppAD::Value(y);
        position.z = CppAD::Value(z);

        return position;
    }

};

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


    // Properties used for optimization management
    Dvector statesNactions; // where all the state and actuation variables will be stored
    Dvector x_lowerbound; //lower limit for each corresponding variable in x
    Dvector x_upperbound; //upper limit for each corresponding variable in x
    Dvector g_lowerbound; // value constraint for each corresponding constraint expression
    Dvector g_upperbound; // value constraint for each corresponding constraint expression

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
        statesNactions.resize(NX);
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


        // object that computes objective and constraints
        FG_eval fg_eval(Traj_ref);

        // options
        std::string options;
        // turn off any printing
        options += "Integer print_level  0\n";
        options += "String  sb           yes\n";
        // maximum number of iterations
        options += "Integer max_iter     10\n";
        // approximate accuracy in first order necessary conditions;
        // see Mathematical Programming, Volume 106, Number 1,
        // Pages 25-57, Equation (6)
        options += "Numeric tol          1e-6\n";
        // derivative testing
        options += "String  derivative_test            second-order\n";
        // maximum amount of random pertubation; e.g.,
        // when evaluation finite diff
        options += "Numeric point_perturbation_radius  0.\n";

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;

        // solve the problem
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options,
            statesNactions,
            x_lowerbound,
            x_upperbound,
            g_lowerbound,
            g_upperbound,
            fg_eval,
            solution);
        
        // Check some of the solution values

        bool ok = true;
        auto cost = solution.obj_value;
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

        if (ok) {
            std::cout << "OK! Cost:" << cost << std::endl;
        } else {
            std::cout << "SOMETHING IS WRONG!" << cost << std::endl;
        }
        // place to return solution

        std::cout << solution.x.size()<< std::endl;

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
        this->g_lowerbound.resize(NG);
        this->g_upperbound.resize(NG);

        // the first constraint for each state variable
        // refer to the initial state conditions
        // this will be initialized when solve() is called
        // the succeeding constraints refer to the relationship
        // between succeeding states based on our kinematic model of the system

        for (int i = 0; i < NG; ++i) {
            this->g_lowerbound[i] = 0.0;
            this->g_upperbound[i] = 0.0;
        }

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

        this->statesNactions[ID_FIRST_px] = px;
        // this->statesNactions[ID_FIRST_py] = py;
        // this->statesNactions[ID_FIRST_pz] = pz;
        // this->statesNactions[ID_FIRST_roll] = roll;
        // this->statesNactions[ID_FIRST_pitch] = pitch;
        // this->statesNactions[ID_FIRST_yaw] = yaw;
        this->statesNactions[ID_FIRST_vx] = 0.0;//vx;
        // this->statesNactions[ID_FIRST_vy] = vy;
        // this->statesNactions[ID_FIRST_vz] = vz;
        // this->statesNactions[ID_FIRST_wx] = wx;
        // this->statesNactions[ID_FIRST_wy] = wy;
        // this->statesNactions[ID_FIRST_wz] = wz;
        // this->statesNactions[ID_FIRST_ax] = vx;
        // this->statesNactions[ID_FIRST_ay] = ay;
        // this->statesNactions[ID_FIRST_az] = az;
        // this->statesNactions[ID_FIRST_alpha_x] = alpha_x;
        // this->statesNactions[ID_FIRST_alpha_y] = alpha_y;
        // this->statesNactions[ID_FIRST_alpha_z] = alpha_z;
    
    }

    // We update the current upper and lower bounds for the constraints
    // The assignation is with its current state value for ensuring a smooth movement
    void update_current_constraint_bounds(){

        this->g_lowerbound[ID_FIRST_px] = -10.0; //this->statesNactions[ID_FIRST_px];
        // this->g_lowerbound[ID_FIRST_py] = -10.0; //this->statesNactions[ID_FIRST_py];
        // this->g_lowerbound[ID_FIRST_pz] = -10.0; //this->statesNactions[ID_FIRST_pz];
        // this->g_lowerbound[ID_FIRST_roll] = this->statesNactions[ID_FIRST_roll];
        // this->g_lowerbound[ID_FIRST_pitch] = this->statesNactions[ID_FIRST_pitch];
        // this->g_lowerbound[ID_FIRST_yaw] = this->statesNactions[ID_FIRST_yaw];
        this->g_lowerbound[ID_FIRST_vx] = this->statesNactions[ID_FIRST_vx];
        // this->g_lowerbound[ID_FIRST_vy] = this->statesNactions[ID_FIRST_vy];
        // this->g_lowerbound[ID_FIRST_vz] = this->statesNactions[ID_FIRST_vz];
        // this->g_lowerbound[ID_FIRST_wx] = this->statesNactions[ID_FIRST_wx];
        // this->g_lowerbound[ID_FIRST_wy] = this->statesNactions[ID_FIRST_wy];
        // this->g_lowerbound[ID_FIRST_wz] = this->statesNactions[ID_FIRST_wz];
        // this->g_lowerbound[ID_FIRST_p_error] = this->statesNactions[ID_FIRST_p_error];
        // this->g_lowerbound[ID_FIRST_roll_error] = this->statesNactions[ID_FIRST_roll_error];
        // this->g_lowerbound[ID_FIRST_pitch_error] = this->statesNactions[ID_FIRST_pitch_error];
        // this->g_lowerbound[ID_FIRST_yaw_error] = this->statesNactions[ID_FIRST_yaw_error];
        // this->g_lowerbound[ID_FIRST_ax] = this->statesNactions[ID_FIRST_ax];
        // this->g_lowerbound[ID_FIRST_ay] = this->statesNactions[ID_FIRST_ay];
        // this->g_lowerbound[ID_FIRST_az] = this->statesNactions[ID_FIRST_az];
        // this->g_lowerbound[ID_FIRST_alpha_x] = this->statesNactions[ID_FIRST_alpha_x];
        // this->g_lowerbound[ID_FIRST_alpha_y] = this->statesNactions[ID_FIRST_alpha_y];
        // this->g_lowerbound[ID_FIRST_alpha_z] = this->statesNactions[ID_FIRST_alpha_z];

        this->g_upperbound[ID_FIRST_px] = 10.0; //this->statesNactions[ID_FIRST_px];
        // this->g_upperbound[ID_FIRST_py] = 10.0; //this->statesNactions[ID_FIRST_py];
        // this->g_upperbound[ID_FIRST_pz] = 10.0; //this->statesNactions[ID_FIRST_pz];
        // this->g_upperbound[ID_FIRST_roll] = this->statesNactions[ID_FIRST_roll];
        // this->g_upperbound[ID_FIRST_pitch] = this->statesNactions[ID_FIRST_pitch];
        // this->g_upperbound[ID_FIRST_yaw] = this->statesNactions[ID_FIRST_yaw];
        this->g_upperbound[ID_FIRST_vx] = this->statesNactions[ID_FIRST_vx];
        // this->g_upperbound[ID_FIRST_vy] = this->statesNactions[ID_FIRST_vy];
        // this->g_upperbound[ID_FIRST_vz] = this->statesNactions[ID_FIRST_vz];
        // this->g_upperbound[ID_FIRST_wx] = this->statesNactions[ID_FIRST_wx];
        // this->g_upperbound[ID_FIRST_wy] = this->statesNactions[ID_FIRST_wy];
        // this->g_upperbound[ID_FIRST_wz] = this->statesNactions[ID_FIRST_wz];
        // this->g_upperbound[ID_FIRST_p_error] = this->statesNactions[ID_FIRST_p_error];
        // this->g_upperbound[ID_FIRST_roll_error] = this->statesNactions[ID_FIRST_roll_error];
        // this->g_upperbound[ID_FIRST_pitch_error] = this->statesNactions[ID_FIRST_pitch_error];
        // this->g_upperbound[ID_FIRST_yaw_error] = this->statesNactions[ID_FIRST_yaw_error];
        // this->g_upperbound[ID_FIRST_ax] = this->statesNactions[ID_FIRST_ax];
        // this->g_upperbound[ID_FIRST_ay] = this->statesNactions[ID_FIRST_ay];
        // this->g_upperbound[ID_FIRST_az] = this->statesNactions[ID_FIRST_az];
        // this->g_upperbound[ID_FIRST_alpha_x] = this->statesNactions[ID_FIRST_alpha_x];
        // this->g_upperbound[ID_FIRST_alpha_y] = this->statesNactions[ID_FIRST_alpha_y];
        // this->g_upperbound[ID_FIRST_alpha_z] = this->statesNactions[ID_FIRST_alpha_z];
    }
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "mpc_custom");
	ros::NodeHandle nh("~");
	mpc_custom mpc;

	ros::spin();
	return 0;
}