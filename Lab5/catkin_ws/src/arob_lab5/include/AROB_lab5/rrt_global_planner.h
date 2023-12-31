/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>

#include <time.h> 
#include "AROB_lab5/TreeNode.h"

#ifndef RRT_PLANNER_CPP
#define RRT_PLANNER_CPP

namespace rrt_planner {

class RRTPlanner : public nav_core::BaseGlobalPlanner {
    
public:

    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // overridden classes from interface nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

private:

    costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
    std::string global_frame_id_;
	bool initialized_;

    double max_samples_;

    double max_dist_;
    double resolution_;

    double min_distanceToGoal;

    // ----- Custom variables -----
    ros::Publisher vis_pub;
    std::vector<visualization_msgs::Marker> original_markers;



    // functions to compute the plan
    bool obstacleFree(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1);
    bool computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol);
    void getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan);

    // ----- Custom methods -----
    std::vector<int> SampleFree();
    std::vector<int> GetConstrainedPoint(const std::vector<int> x1, const std::vector<int> x2, const double dist_to_x1);
    std::vector<int> InputNSteer(const std::vector<int> xnear, const std::vector<int> xrand);
    void printMarker(std::vector<double> pose, int id);
    void printPlan(const std::vector<geometry_msgs::PoseStamped> plan, const geometry_msgs::PoseStamped& start);
    void deleteMarkers();
    void deleteMarker(visualization_msgs::Marker original_marker);

};

};
 #endif
