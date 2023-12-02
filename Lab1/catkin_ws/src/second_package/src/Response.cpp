#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <string.h>


// Publisher node global for publishing from the callback
ros::Publisher chatter_pub;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  // prints what has listened (activated the callback)
  ROS_INFO("I heard: [%s]", msg->data.c_str());


  // Searches in the received string "Autonomous Robots Course"
  std::size_t found = msg->data.find("Autonomous Robots Course");
  // If it is contained in the msg string
  if (found!=std::string::npos){
    // Creates and publish the "Hello, I am a student" message
    std::stringstream ss;
    ss << "Hello, I am a student!";
    std_msgs::String response_msg;
    response_msg.data = ss.str();

    ROS_INFO("%s", response_msg.data.c_str());

    chatter_pub.publish(response_msg);
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
   
   // Creates and launches the nodes
   ros::init(argc, argv, "responser_listen");
   ros::init(argc, argv, "responser_talk");

	// Creates the node hanler that manages the nodes
	ros::NodeHandle n;
	
	// Assign the node chatter_pub to be a publisher in the topic "chatter"
	chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	// Assign the node sub to be a subscriber in the topic "chatter" and launches the chatterCallback each time that a message is published in the "chatter" topic
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


 /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
