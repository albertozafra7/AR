#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class FollowTargetsClass {
	ros::NodeHandle nh_;
	ros::Publisher goal_pub_;
	ros::Subscriber position_sub_;
	geometry_msgs::PoseStamped Goal;
	ifstream inFile;
	std::vector<std::vector<float> > targets;
	int currentTarget; //index with the next target to reach
	bool end_reached;


public:
	FollowTargetsClass() { //in the contructor you can read the targets from the text file
	
		// Subscribe to the /base_pose_groun_truth topic
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &FollowTargetsClass::UpdateGoal, this);
		// Assign the node goal to be a publisher in the topic "goal"
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		
		std::string packagePath = ros::package::getPath("p12_arob_lab2");
		//ROS_INFO("PackagePath: %s", packagePath.c_str());
		loadTargetsFromFile(packagePath+"/src/targets.txt");
		
		end_reached = false;
		GetNewGoal();
	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need

	void loadTargetsFromFile(const std::string& filename) {
		inFile.open(filename);

		if (!inFile.is_open()) {
			std::cerr << "Error: Unable to open the file " << filename << std::endl;
		} else {
			std::string line;
			int i = 0;
			while (std::getline(inFile, line)) {
				// Parse the line to extract x and y values
				std::istringstream iss(line);
				char delimiter = ';';
				std::vector<float> target(2);

				if (iss >> target[0] >> delimiter >> target[1]) {
					targets.push_back(target);
				} else {
					std::cerr << "Warning: Invalid format in line: " << line << std::endl;
				}
			}
			currentTarget = 0;
			inFile.close();
		}
	}

	void GetNewGoal() {

		if(!targets.empty()){
			std::vector<float> target = targets.front();
			targets.erase(targets.begin());

			std::cout << " Goal Update: ("<< target[0] << ", " << target[1] << ")" << endl;
			
			//	update the goal
			Goal.pose.position.x = target[0];
			Goal.pose.position.y = target[1];
			
			
			//std::cout << " Goal= ("<< Goal.pose.position.x << ", " << Goal.pose.position.y << ")" << endl;
			// pubilsh the new goal
			goal_pub_.publish(Goal);
		} else if(!end_reached){
			std::cout << "Audrie has passed by all the targets, please add more difficulty to the circuit!!!" << endl;
			end_reached = true;
		}
	}

	void UpdateGoal(const nav_msgs::Odometry& msg) {
		float ex = Goal.pose.position.x - msg.pose.pose.position.x;
		float ey = Goal.pose.position.y - msg.pose.pose.position.y;
		
		if(std::abs(ex) + std::abs(ey) < 0.2) {
			if(!targets.empty()){
				std::cout << endl << endl << "Target " << currentTarget << " Reached!!!" << endl << "Moving to the next target..." << endl;
				currentTarget++;
			}
			GetNewGoal();
		}
	
	}


};


int main(int argc, char** argv) {


	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh("~");
	FollowTargetsClass FT;
	

	ros::spin();
	return 0;
}

