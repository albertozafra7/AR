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

class FollowTargets3DClass {
	ros::NodeHandle nh_;
	ros::Publisher goal_pub_;
	ros::Subscriber position_sub_;
	geometry_msgs::PoseStamped Goal;
	ifstream inFile;
	std::vector<std::vector<float> > targets;
	int currentTarget; //index with the next target to reach
	bool end_reached;
	int file_id = 0;
	
	std::vector<double> prev_pos = {0,0,0};


public:
	FollowTargets3DClass(int argv_file_id) { //in the contructor you can read the targets from the text file
	
		// Subscribe to the /base_pose_groun_truth topic
		position_sub_ = nh_.subscribe("/ground_truth/state", 1, &FollowTargets3DClass::UpdateGoal, this);
		// Assign the node goal to be a publisher in the topic "goal"
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/command/pose", 1, true);
		
		std::string packagePath = ros::package::getPath("p12_arob_lab3");
		//ROS_INFO("PackagePath: %s", packagePath.c_str());
		// We load the 6 files
		file_id = argv_file_id;
		evaluate_file_id(packagePath);
		end_reached = false;
		GetNewGoal();
	}

	~FollowTargets3DClass() {
	}
	
	
	void evaluate_file_id(std::string packagePath){
			switch(file_id){
			case 1:
				loadTargetsFromFile(packagePath+"/src/targets.txt");
				break;
			case 2:
				loadTargetsFromFile(packagePath+"/src/targets2.txt");
				break;
			case 3:
				loadTargetsFromFile(packagePath+"/src/targets3.txt");
				break;
			case 4:
				loadTargetsFromFile(packagePath+"/src/targets4.txt");
				break;
			case 5:
				loadTargetsFromFile(packagePath+"/src/targets5.txt");
				break;
			case 6:
				loadTargetsFromFile(packagePath+"/src/targets6.txt");
				break;
			default:
				loadTargetsFromFile(packagePath+"/src/targets.txt");
		}
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
				std::vector<float> target(3);

				if (iss >> target[0] >> delimiter >> target[1] >> delimiter >> target[2]) {
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

			std::cout << " Goal Update: ("<< target[0] << ", " << target[1] << ", " << target[2] << ")" << endl;
			
			//	update the goal
			Goal.pose.position.x = static_cast<float>(target[0]);
			Goal.pose.position.y = static_cast<float>(target[1]);
			Goal.pose.position.z = static_cast<float>(target[2]);
			
			
			//std::cout << " Goal= ("<< Goal.pose.position.x << ", " << Goal.pose.position.y << ")" << endl;
			// pubilsh the new goal
			goal_pub_.publish(Goal);
		} else if(!end_reached){
			std::cout << "Padron has passed by all the targets, please add more difficulty to the circuit!!!" << endl;
			end_reached = true;
		}
	}

	void UpdateGoal(const nav_msgs::Odometry& msg) {
		//float ex = Goal.pose.position.x - msg.pose.pose.position.x;
		//float ey = Goal.pose.position.y - msg.pose.pose.position.y;
		//float ez = Goal.pose.position.z - msg.pose.pose.position.z;
		
		std::vector<double> current_pos{msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z};
		std::vector<double> goal_pos{Goal.pose.position.x,Goal.pose.position.y,Goal.pose.position.z};

		// If the robot is close to the goal move to the next target
		if(euclidean_dist(current_pos,goal_pos)  < 0.2) {
			if(!targets.empty()){
				std::cout << endl << endl << "Target " << currentTarget << " Reached!!!" << endl << "Moving to the next target..." << endl;
				currentTarget++;
			}
			GetNewGoal();
		}
		

		// If the robot is not moving, reload the goal, because it might be due to a bad publication in the topic
		// This has been solved by stablishing the latch parameter on the publisher to true
		//if(!isMoving(current_pos))
			//ReloadGoal();

		// Update previous position
		prev_pos = current_pos;
		
		
	
	}
	
	// Euclidean distance calculus
	float euclidean_dist(std::vector<double> origin, std::vector<double> goal){
		float ex = goal[0] - origin[0];
		float ey = goal[1] - origin[1];
		float ez = goal[2] - origin[2];
		
		float dist = sqrt(pow(ex,2)+pow(ey,2)+pow(ez,2));
		
		return dist;
	}
	
	// Checks if the robot is moving by comparing the previous position with the current position
	bool isMoving(std::vector<double> current_pos){
	
			if(euclidean_dist(current_pos,prev_pos)  < 0.2)
				return false;
			else
				return true;
	
	}
	
	
	// Re-Publish the goal, in case that the robot didn't receive it correctly
	// Normally called only for the first goal
	void ReloadGoal(){
		goal_pub_.publish(Goal);
	}

};


int main(int argc, char** argv) {

	int file_id = 0;
	if(argc > 1)
		file_id = stoi(argv[1]);

	ros::init(argc, argv, "followTargets3D");
	ros::NodeHandle nh("~");
	FollowTargets3DClass FT(file_id);
	

	ros::spin();
	return 0;
}

