#include "arob_mpc/mpc_custom.h"

using namespace std;


class mpc_custom {
	ros::NodeHandle nh_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
	ros::Subscriber goal_sub_;

	std::vector<geometry_msgs::PoseStamped> Traj_goal;  // Line to follow
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

        Traj_goal = msg.poses;

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