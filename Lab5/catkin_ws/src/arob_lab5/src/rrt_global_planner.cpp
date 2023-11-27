#include <pluginlib/class_list_macros.h>
#include "AROB_lab5/rrt_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace rrt_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

double distance(const std::vector<int> x0, const std::vector<int> x1){
    return std::sqrt((x1[0]-x0[0])*(x1[0]-x0[0]) + (x1[1]-x0[1])*(x1[1]-x0[1]));
}

// Vector module
double vectModule(const std::vector<int> vect){
    return std::sqrt(std::pow(vect[0],2)+std::pow(vect[1],2));
}

// Normalization of the vector
std::vector<double>normalize(const std::vector<int> vect){
    double mod = vectModule(vect);
    return {vect[0]/mod,vect[1]/mod};
}


RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0){}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    if (!initialized_){
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle nh_local("~/local_costmap/");
        ros::NodeHandle nh_global("~/global_costmap/");

        nh.param("maxsamples", max_samples_, 0.0);

        //to make sure one of the nodes in the plan lies in the local costmap
        double width, height;
        nh_local.param("width", width, 3.0);
        nh_local.param("height", height, 3.0);
        max_dist_ = (std::min(width, height)/3.0);  //or any other distance within local costmap

        nh_global.param("resolution", resolution_, 0.05); //meters/cell
        nh_global.param("min_distanceToGoal", min_distanceToGoal, 0.2);//minimum distance to goal

        // std::cout << "Parameters: " << max_samples_ << ", " << dist_th_ << ", " << visualize_markers_ << ", " << max_dist_ << std::endl;
        // std::cout << "Local costmap size: " << width << ", " << height << std::endl;
        // std::cout << "Global costmap resolution: " << resolution_ << std::endl;

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros_->getGlobalFrameID();

        initialized_ = true;
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    // std::cout << "RRTPlanner::makePlan" << std::endl;
    
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized.");
        return false;
    }

	if (start.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), start.header.frame_id.c_str());
		return false;
	}

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();  // Update information from costmap
    
    // Get start and goal poses in map coordinates
    unsigned int goal_mx, goal_my, start_mx, start_my;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
        ROS_WARN("Goal position is out of map bounds.");
        return false;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRT;
    bool computed = computeRRT(point_start, point_goal, solRRT);
    if (computed){        
        getPlan(solRRT, plan);
        // add goal
        plan.push_back(goal);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}


std::vector<int> RRTPlanner::SampleFree(){
    // Get the new random location
    std::vector<int> xrand{rand(),rand()};
    unsigned int xrand_mx, xrand_my;

    // While the computed random location is not in a free space and if the world coordinates are not within the map
    while(costmap_->getCost(xrand[0], xrand[1]) == costmap_2d::FREE_SPACE && !costmap_->worldToMap(xrand[0], xrand[1], xrand_mx, xrand_my)){
        // We compute new random locations
        xrand[0] = rand();
        xrand[1] = rand();
    }
    return xrand; 
}

// Get the closest node in the tree to the new location with the euclidean distance
TreeNode* RRTPlanner::Nearest(const std::vector<int> xrand, TreeNode* itr_node){
    double min_distance = distance(xrand,itr_node->getNode());
    double child_distance = 0.0f;
    TreeNode* xnear = itr_node;   // The node that we will return

    if(itr_node->hasChildren()){
        for(int i = 0; i < itr_node->childrenNumber(); i++){
            child_distance = distance(xrand,itr_node->getChild(i)->getNode());
            if(child_distance < min_distance){
                min_distance = child_distance;
                xnear = itr_node->getChild(i);
            }
            // If the child has children also we run the method recursivelly
            if(itr_node->getChild(i)->hasChildren())
                xnear = Nearest(xrand, itr_node->getChild(i));
                
        }

    }
    return xnear; 
}

// Obtain a point between x1 and x2 at a fixed distance to x1
std::vector<int> RRTPlanner::GetConstrainedPoint(const std::vector<int> x1, const std::vector<int> x2, const double dist_to_x1){
    //std::vector<int> dir_vect = {x2[0]-x1[0],x2[1]-x1[1]};  // director vector between x1 and x2
    std::vector<double> norm_vect = normalize({x2[0]-x1[0],x2[1]-x1[1]}); // Normalized director vector between x1 and x2
    
    // The new point is x = x1 + dist * norm_vect
    std::vector<int> x = {(x1[0] + static_cast<int>(dist_to_x1 * norm_vect[0])), (x1[1] + static_cast<int>(dist_to_x1 * norm_vect[1]))};
    
    return x;
}


// Modify xrand in order to get a node that fulfills the movement constraints
std::vector<int> RRTPlanner::InputNSteer(const std::vector<int> xnear, const std::vector<int> xrand){
    // In this case the only constraint is the maximum distance reachable from xnear
    std::vector<int> xnew = xrand;
    if(distance(xnear,xrand) > max_dist_)
        xnew = GetConstrainedPoint(xnear, xrand, max_dist_);

    return xnew;
}

bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol){
    bool finished = false;

    //Initialize random number generator
    srand(time(NULL));
        
    // Initialize the tree with the starting point in map coordinates
    TreeNode *itr_node = new TreeNode(start); 
    TreeNode *closest_node = itr_node;

    double n_samples = 0;

    // while goal not reachable
    while((distance(closest_node->getNode(),goal) > min_distanceToGoal) && n_samples <= max_samples_) {
        // Xrand = SampleFree
        std::vector<int> xrand = SampleFree();
        
        // Xnear = Nearest(G=(V,E),xrand)
        TreeNode* xnear = Nearest(xrand, itr_node);

        // u = input(xnear,xrand)
        // xnew = steer(xnear,u)
        std::vector<int> xnew = InputNSteer(xnear->getNode(),xrand);

        // if ObstacleFree(xnear,xnew)
        if(obstacleFree(xnear->getNode()[0],xnear->getNode()[1],xnew[0],xnew[1]))
            // V = V union xnew
            // E = E union {(xnear,xnew)}
            xnear->appendChild(new TreeNode(xnew));

        closest_node = Nearest(goal,itr_node);
        sol = closest_node->returnSolution();
    }
    

    if(n_samples <= max_samples_)
        finished = true;

    // Return G
    // implement RRT here!


    itr_node->~TreeNode();

    return finished;
}

bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE){  // to include cells with inflated cost
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}

void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
        std::vector<int> point = (*it);
        geometry_msgs::PoseStamped pose;

        costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_id_;
        pose.pose.orientation.w = 1;
        plan.push_back(pose);

    }
}

};
