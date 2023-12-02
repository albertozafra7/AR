#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char** argv){

	// We launch the node "robot_location"
	ros::init(argc, argv, "robot_location");
	ros::NodeHandle node;
	
	// We determine that we are going to become a subscriber of the "tf" topic with a frequency of 2Hz
	tf::TransformListener listener;
	ros::Rate rate(2.0);
	listener.waitForTransform("/base_footprint", "/odom", ros::Time(0),ros::Duration(10.0));
	
	while (ros::ok()){
	
		tf::StampedTransform transform;
		
		try {
		
			// We receive the /base_footprint coordinate frame based on the /odom reference system and we store it in the transform struct
			listener.lookupTransform("/base_footprint", "/odom", ros::Time(0),transform);
			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			cout << "Current position: (" << x << "," << y << ")" << endl;
			
		} catch (tf::TransformException &ex) {
		
			ROS_ERROR("%s",ex.what());
			
		}
		// why sleep?
		rate.sleep();
	}
	return 0;
}
