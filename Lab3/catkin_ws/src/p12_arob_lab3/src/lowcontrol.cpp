#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <stdio.h> 
#include <math.h>
#include <fstream>
#include <tf/transform_broadcaster.h>

using namespace std;


class Lowlevelcontrol {
	ros::NodeHandle nh_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
	ros::Subscriber goal_sub_;
	geometry_msgs::PoseStamped Goal;
	float krho, kalpha, kbeta;
public:
	Lowlevelcontrol() {

		// Subscribe to the /base_pose_groun_truth topic
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &Lowlevelcontrol::positionCb, this);
		// Subscribe to the /goal topic
		goal_sub_ = nh_.subscribe("goal", 1, &Lowlevelcontrol::goalCb, this);
		// Sets as publisher to the velocity topic
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
		Goal.pose.position.x = 0; //update the initial values of these variables
		Goal.pose.position.y = 0;
		// kalpha - krho > 0
		kalpha = 0.25; // the values of these parameters should be obtained by you
		krho = 0.2; // krho > 0
		kbeta = -0.001; // kbeta < 0
	}

	~Lowlevelcontrol() {
	}

	void goalCb(const geometry_msgs::PoseStamped& msg) {


		std::cout << "lowcontrol has received a Goal Update: ("<< msg.pose.position.x << ", " << msg.pose.position.y << ")" << endl;

		//	update the goal
		Goal.pose.position.x = msg.pose.position.x;
		Goal.pose.position.y = msg.pose.position.y;
	}

	void positionCb(const nav_msgs::Odometry& msg) {

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
	}

	
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "lowcontrol");
	ros::NodeHandle nh("~");
	Lowlevelcontrol llc;

	ros::spin();
	return 0;
}
