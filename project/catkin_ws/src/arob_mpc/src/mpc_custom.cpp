#include "arob_mpc/mpc_custom.h"

using namespace std;


class mpc_custom {
	ros::NodeHandle nh_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
	ros::Subscriber goal_sub_;

	std::vector<geometry_msgs::PoseStamped> Traj_ref;  // Line to follow
    int lookahead; // lookahead (aka size of the traj_goal)

public:
	mpc_custom() {

		// Subscribe to the /base_pose_groun_truth topic
		//position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &mpc_custom::positionCb, this);
		// Subscribe to the /goal_pos topic that give us the trajectory to follow with a lookahead
		goal_sub_ = nh_.subscribe("goal_pos", 1, &mpc_custom::goal_update, this);
		// Sets as publisher to the velocity topic
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


		// kalpha - krho > 0
		// kalpha = 0.25; // the values of these parameters should be obtained by you
		// krho = 0.2; // krho > 0
		// kbeta = -0.001; // kbeta < 0
	}

	~mpc_custom() {
	}

    void goal_update(const arob_mpc::vector_poses& msg){

        ROS_INFO("The Custom MPC has received a new trajectory, with a lookahead of %ld to follow", msg.poses.size());

        Traj_ref = msg.poses;

        // std::cout << "Printing received point :" << std::endl;

        // print_vector(msg.poses);

        // std::cout << "Printing overwritten vector :" << std::endl;

        // print_vector(Traj_goal);


    }


    void position_update(const nav_msgs::Odometry& msg){

        // Get the positions
		float px = msg.pose.pose.position.x;
		float py = msg.pose.pose.position.y;
		float pz = msg.pose.pose.position.z;
        

        // Get the errors

        // Call the mpc method

        // Publish the new velocities obtained from the mpc

    }

    
    void mpc(){


        // place to return solution
        // CppAD::ipopt::solve_result<Dvector> solution;
        // CppAD::ipopt::solve
        // Get the errors and stablish the constraints

        // call ipopt to solve optimize the mpc

        // return the velocities
    }

    AD<double> costfunction(const ADvector& statesNactions){
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

    // void print_vector(std::vector<geometry_msgs::PoseStamped> vector){
    //     for(int i = 0; i < vector.size(); ++i){
    //         std::cout << "Printing pose " << i << std::endl;
    //         print_pose(vector[i].pose.position);
    //     }
    // }

    // void print_pose(geometry_msgs::Point pose){
    //     std::cout << "\t" << "(" << pose.x << ", " << pose.y << ", " << pose.z << ")" << std::endl;
    // }

	/*void positionCb(const nav_msgs::Odometry& msg) {

		float ex = Goal.pose.position.x - msg.pose.pose.position.x;
		float ey = Goal.pose.position.y - msg.pose.pose.position.y;
		float rho = sqrt(ex*ex+ey*ey);
		float alpha = atan2(ey,ex) - tf::getYaw(msg.pose.pose.orientation);
		if (alpha < -M_PI) alpha = 2*M_PI - abs(alpha);
		if (alpha > M_PI) alpha = -2*M_PI + alpha;
		float beta = - alpha - tf::getYaw(msg.pose.pose.orientation);
		if (beta < -M_PI) beta = 2*M_PI - abs(beta);
		if (beta > M_PI) beta = -2*M_PI + beta;
		
		//std::cout << "Goal: " << Goal.pose.position << endl;
		//std::cout << "Robot: " << msg.pose.pose.position << endl;
		
		//std::cout << "ex: "<< ex << " ";
		//std::cout << "ey: "<< ey << " ";

		//std::cout << "Rho: "<< rho << " ";
		//std::cout << "Alpha: "<< alpha << " ";
		//std::cout << "Beta: "<< beta << endl;

		//std::cout << "X: "<< msg.pose.pose.position.x << " ";
		//std::cout << "Y: "<< msg.pose.pose.position.y << " ";
		//std::cout << "Th: "<< tf::getYaw(msg.pose.pose.orientation) << endl;

		geometry_msgs::Twist input; //to send the velocities
		
		if(std::abs(ex) + std::abs(ey) > 0.2){
			input.linear.x = krho*rho;
			input.linear.x = (input.linear.x >= 1.0) ? 0.95 : input.linear.x;
			//input.linear.y = ey*krho*rho;
		
			//input.angular.x = kalpha*alpha + kbeta*beta;
			input.angular.z = kalpha*alpha + kbeta*beta;
		} else {
			input.linear.x = 0.0;
			//input.linear.y = ey*krho*rho;
		
			//input.angular.x = kalpha*alpha + kbeta*beta;
			input.angular.z = 0.0;
		}
		//std::cout << "v: " << input << endl;
		

		//here you have to implement the controller
		velocity_pub_.publish(input);
	}*/

	
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "lowcontrol");
	ros::NodeHandle nh("~");
	mpc_custom mpc;

	ros::spin();
	return 0;
}