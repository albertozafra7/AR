#include "arob_mpc/mpc_custom.h"

using namespace std;

// +++++++++++++++++++++++++++ MPC (IPOPT OPTIMIZATION) +++++++++++++++++++++++++++
class FG_eval {

public:
    // Fitted polynomial coefficients
    std::vector<geometry_msgs::PoseStamped> Traj_ref;  // Line to follow

    FG_eval(std::vector<geometry_msgs::PoseStamped> Traj_in) : Traj_ref(Traj_in) {}

    void operator()(ADvector& fg, const ADvector& vars) {

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
    void states_prediction(ADvector& statesNactions){
        // Remember that the system model is the following:
            // px_k+1 = px_k + timestep * vx_k
            // py_k+1 = py_k + timestep * vy_k
            // pz_k+1 = pz_k + timestep * vz_k
            // roll_k+1 = roll_k + timestep * wx_k
            // pitch_k+1 = pitch_k + timestep * wy_k
            // yaw_k+1 = yaw_k + timestep * wz_k

            // Same for velocities

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
	ros::Subscriber goal_sub_;

	std::vector<geometry_msgs::PoseStamped> Traj_ref;  // Line to follow
    //int lookahead; // lookahead (aka size of the traj_goal)

    geometry_msgs::PoseStamped current_pose; // Current pose of the quadrotor

public:
	mpc_custom() {

		// Subscribe to the /base_pose_groun_truth topic
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &mpc_custom::pose_update, this);
		// Subscribe to the /goal_pos topic that give us the trajectory to follow with a lookahead
		goal_sub_ = nh_.subscribe("goal_pos", 1, &mpc_custom::goal_update, this);
		// Sets as publisher to the velocity topic
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	}

	~mpc_custom() {
	}

    void goal_update(const arob_mpc::vector_poses& msg){

        ROS_INFO("The Custom MPC has received a new trajectory, with a lookahead of %ld to follow", msg.poses.size());

        // Update the poses
        Traj_ref = msg.poses;

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
		current_pose.pose.position.x = msg.pose.pose.position.x;
		current_pose.pose.position.y = msg.pose.pose.position.y;
		current_pose.pose.position.z = msg.pose.pose.position.z;

        current_pose.pose.orientation.x = msg.pose.pose.orientation.x;
        current_pose.pose.orientation.y = msg.pose.pose.orientation.y;
        current_pose.pose.orientation.z = msg.pose.pose.orientation.z;
        current_pose.pose.orientation.w = msg.pose.pose.orientation.w;
    }

    
    void mpc(){

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
        // CppAD::ipopt::solve<Dvector, FG_eval>(
        //     options,
        //     x,
        //     x_lowerbound,
        //     x_upperbound,
        //     g_lowerbound,
        //     g_upperbound,
        //     fg_eval,
        //     solution);
        //
        // Check some of the solution values
        //

        bool ok = true;
        auto cost = solution.obj_value;
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

        if (ok) {
            std::cout << "OK! Cost:" << cost << std::endl;
        } else {
            std::cout << "SOMETHING IS WRONG!" << cost << std::endl;
        }
        // place to return solution


        // return the velocities
    }
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "lowcontrol");
	ros::NodeHandle nh("~");
	mpc_custom mpc;

	ros::spin();
	return 0;
}