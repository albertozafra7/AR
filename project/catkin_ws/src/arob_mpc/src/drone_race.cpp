#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <sstream>
#include <stdio.h> 
#include <math.h>
#include <fstream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <ros/package.h>

#include "arob_mpc/vector_poses.h"

using namespace std;

class drone_race {

    std::vector<geometry_msgs::Pose> gates;

    //Trajectory attributes
    mav_trajectory_generation::Trajectory trajectory;

    //ROS publishers-suscribers
    ros::NodeHandle nh_;
    ros::Publisher pub_traj_markers_;
    ros::Publisher pub_traj_vectors_;
    ros::Publisher pub_gate_markers_;
    

    //Id markers
    int id_marker = 0;

    // Custom properties
    ros::Publisher pub_drone_vel_;   // Drone velocity publisher
    ros::Publisher pub_drone_pos_;
    std::vector<geometry_msgs::Twist> drone_vel_list;   // List of velocities to send to the drone
    std::vector<geometry_msgs::PoseStamped> drone_pose_list; // List of poses to send to the mpc
    std::vector<geometry_msgs::Accel> drone_accel_list; // List of accelerations
    int global_it = 0;

    public:

    drone_race() {

        // create publisher for RVIZ markers
        pub_traj_markers_ =
            nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
        pub_traj_vectors_ =
            nh_.advertise<visualization_msgs::MarkerArray>("trajectory_vectors", 0);
        pub_gate_markers_ =
            nh_.advertise<visualization_msgs::MarkerArray>("gate_markers", 0);
            
        // Create publisher for gazebo
        pub_drone_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        pub_drone_pos_ = nh_.advertise<arob_mpc::vector_poses>("goal_pos", 1, true);
        global_it = 0;
	}

    ~drone_race() {
    }

    void resetGlobalIt(){
        global_it = 0;
    }

    void incrementGlobalIt(){
        global_it++;
    }

    int readGates(string file) {
        //Open the file
        ifstream inputFile;
	    inputFile.open(file, ifstream::in);
	    if (!inputFile) {
        	cerr << "Error opening the file." << endl;
        	return -1;
	    }

        gates.clear();
        geometry_msgs::Pose tempPose;
        double yaw = 0;
        std::string line;
        while (std::getline(inputFile, line))
        {
            std::istringstream iss(line);
            iss >> tempPose.position.x;
            iss >> tempPose.position.y;
            iss >> tempPose.position.z;
            iss >> yaw;
            tempPose.orientation = RPY_to_quat(0, 0, yaw);
            gates.push_back(tempPose);
        }

        // Close the file
        inputFile.close();
        return 1;
    }

    int drawGates() {
        int id = 0;
        for (geometry_msgs::Pose gate : gates) {
            draw_gate_markers(gate);
        }
        return 1;
    }

    void generate_trajectory_example() {
        //constants
        const int dimension = 3; //we only compute the trajectory in x, y and z
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP

        // Definition of the trajectory beginning, end and intermediate constraints
        mav_trajectory_generation::Vertex::Vector vertices;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
        start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);   // We set all the derivatives to 0
        vertices.push_back(start);

        //Position constraint
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3)); // We can specify specific values to the  derivatives (position)
        //Velocity constraint (optional)
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(1,0,0)); //specify the velcity in this point to be (1, 0, 0)
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
        vertices.push_back(end);
        
        // Provide the time constraints on the vertices
        //Automatic time computation
        std::vector<double> segment_times; //we'll need n - 1 segment times, n = points of the path
        const double v_max = 2.0;
        const double a_max = 2.0;
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);
        cout << "Segment times = " << segment_times.size() << endl;
        /*for (int i=0; i< segment_times.size() ; i++) {
            cout << "Time " << i << " = " << segment_times[i] << endl;
        }*/
        //Manual time computation
        segment_times.clear();
        segment_times.push_back(3.5); // This is the time required to go from vertex 0 to vertex 1
        segment_times.push_back(2.5); // This is the time required to go from vertex 1 to vertex 2
        
        // Solve the optimization problem
        const int N = 10; //Degree of the polynomial, even number at least two times the highest derivative
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        //Obtain the trajectory
        opt.getTrajectory(&trajectory); // Here we get the optimized trajectory (the function that determines how to pass through the gates)
        //Sample the trajectory (to obtain positions, velocities, etc.)
        mav_msgs::EigenTrajectoryPoint::Vector states;
        double sampling_interval = 0.01; //How much time between intermediate points
        // We define a sampling interval and we get the vector with the intermediate positions/velocities/accel... at specific times
        bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
        // Example to access the data
        cout << "Trajectory time = " << trajectory.getMaxTime() << endl;
        cout << "Number of states = " << states.size() << endl;
        cout << "Position (world frame) " << 3 << " X = " << states[2].position_W[0] << endl;   // X position after 20ms (2*sampling_interval)
        cout << "Velocity (world frame) " << 3 << " X = " << states[2].velocity_W[0] << endl;

        // Default Visualization
        visualization_msgs::MarkerArray markers;
        double distance = 0.25; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        pub_traj_vectors_.publish(markers);

        //AROB visualization
        draw_trajectory_markers();
    }

    void generate_trajectory() {
        //constants
        const int dimension = 3; //we only compute the trajectory in x, y and z
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP

        // Definition of the trajectory beginning, end and intermediate constraints
        mav_trajectory_generation::Vertex::Vector vertices;
        mav_trajectory_generation::Vertex start_vertex(dimension), end_vertex(dimension);
        std::vector<mav_trajectory_generation::Vertex> middle_vertices;
        
        Eigen::Matrix<double, 3, 1> local_vel_constraint(2.3, 0.0, 0.0);
        

        // Start config
        start_vertex.makeStartOrEnd(Eigen::Vector3d(0,0,0), derivative_to_optimize);
        vertices.push_back(start_vertex);

        // Creation of the intermediate points
        for(int i = 0; i < gates.size(); i++){
            middle_vertices.push_back(mav_trajectory_generation::Vertex(dimension));
            // Position constraints
            //std::cout << gates[i].position << std::endl;
            middle_vertices[i].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(gates[i].position.x,gates[i].position.y,gates[i].position.z));
            // Velocity constraints
            Eigen::Matrix<double, 3, 1> global_vel_constraint = quat_to_R_matrix(gates[i].orientation) * local_vel_constraint;
            middle_vertices[i].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(global_vel_constraint(0),global_vel_constraint(1),global_vel_constraint(2)));
            // Add to vertices 
            vertices.push_back(middle_vertices[i]);
        }

        // End config
        end_vertex.makeStartOrEnd(Eigen::Vector3d(0,0,0), derivative_to_optimize);
        vertices.push_back(end_vertex);


        // Provide the time constraints on the vertices
        //Automatic time computation
        std::vector<double> segment_times; //we'll need n - 1 segment times, n = points of the path
        const double v_max = 2;// 4.5;
        const double a_max = 2;// 8.65;
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);
        cout << "Segment times = " << segment_times.size() << endl;
        for (int i=0; i< segment_times.size() ; i++) {
            cout << "Time " << i << " = " << segment_times[i] << endl;
        }
        //Manual time computation
        /*segment_times.clear();
        segment_times.push_back(3.5); // This is the time required to go from vertex 0 to vertex 1
        segment_times.push_back(2.5); // This is the time required to go from vertex 1 to vertex 2*/
        
        
        // Solve the optimization problem
        const int N = 10; //Degree of the polynomial, even number at least two times the highest derivative
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        //Obtain the trajectory
        trajectory.clear();
        opt.getTrajectory(&trajectory);
        //Sample the trajectory (to obtain positions, velocities, etc.)
        mav_msgs::EigenTrajectoryPoint::Vector states;
        double sampling_interval = 0.1; //How much time between intermediate points
        bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
        // Example to access the data
        cout << "Trajectory time = " << trajectory.getMaxTime() << endl;
        cout << "Number of states = " << states.size() << endl;
        //cout << "Position (world frame) " << 3 << " X = " << states[0].position_W[0] << endl;
        //cout << "Velocity (world frame) " << 3 << " X = " << states[0].velocity_W[0] << endl;
        
        // Default Visualization
        visualization_msgs::MarkerArray markers;
        double distance = 0.25; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        pub_traj_vectors_.publish(markers);

        //AROB visualization
        draw_trajectory_markers();

        // Generate list of commands to publish to the drone 
        // We send a command to the drone every 100ms (take in mind)
        for(int i = 0; i < states.size(); i++){
            // Velocity push_back
            geometry_msgs::Twist temp_vel;

            temp_vel.linear.x = states[i].velocity_W[0];
            temp_vel.linear.y = states[i].velocity_W[1];
            temp_vel.linear.z = states[i].velocity_W[2];

            temp_vel.angular.x = states[i].angular_velocity_W[0];
            temp_vel.angular.y = states[i].angular_velocity_W[1];
            temp_vel.angular.z = states[i].angular_velocity_W[2];

            drone_vel_list.push_back(temp_vel);


            // Position push_back
            geometry_msgs::PoseStamped temp_pos;

            temp_pos.pose.position.x = states[i].position_W[0];
            temp_pos.pose.position.y = states[i].position_W[1];
            temp_pos.pose.position.z = states[i].position_W[2];


            temp_pos.pose.orientation.x = states[i].orientation_W_B.x();
            temp_pos.pose.orientation.y = states[i].orientation_W_B.y();
            temp_pos.pose.orientation.z = states[i].orientation_W_B.z();
            temp_pos.pose.orientation.w = states[i].orientation_W_B.w();

            drone_pose_list.push_back(temp_pos);


            // Acceleration push_back
            geometry_msgs::Accel temp_accel;

            temp_accel.linear.x = states[i].acceleration_W.x();
            temp_accel.linear.y = states[i].acceleration_W.y();
            temp_accel.linear.z = states[i].acceleration_W.z();

            temp_accel.angular.x = states[i].angular_acceleration_W.x();
            temp_accel.angular.y = states[i].angular_acceleration_W.y();
            temp_accel.angular.z = states[i].angular_acceleration_W.z();

            drone_accel_list.push_back(temp_accel);

        }

        //send_command();

        
        // send_goals(N); // Look ahead trajectory steps
        const int lookahead = 10;

        send_states(lookahead);
        

    }

    void send_command() {
        // Sending position goals (easier but bad option in pratice) or velocity commands ()
        // Publish each velocity command in the list
        for (const auto& vel_command : drone_vel_list) {
            //std::cout << vel_command << std::endl;
            pub_drone_vel_.publish(vel_command);
            ros::Duration(0.1).sleep(); 
        }
    }

    void send_goals(int lookahead) {
        // Sending position goals (easier but bad option in pratice) or velocity commands ()
        // Publish each position command in the list with a prediction lookahead
        arob_mpc::vector_poses lookahead_poses;
        for (int i = 0; i < drone_pose_list.size(); ++i) {
            std::vector<geometry_msgs::PoseStamped> temp_poses;
            for (int j = 0; j < min(lookahead, static_cast<int>(drone_pose_list.size()-i)); ++j)
                temp_poses.push_back(drone_pose_list[i+j]);
            //std::cout << vel_command << std::endl;
            lookahead_poses.poses = temp_poses;
            pub_drone_pos_.publish(lookahead_poses);
            ros::Duration(0.1).sleep(); 
            temp_poses.clear();
        }
    }

    void send_states(int lookahead) {
        // Sending position goals (easier but bad option in pratice) or velocity commands ()
        // Publish each position command in the list with a prediction lookahead
        arob_mpc::vector_poses lookahead_states;
        for (int i = 0; i < drone_pose_list.size(); ++i) {
            std::vector<geometry_msgs::PoseStamped> temp_poses;
            std::vector<geometry_msgs::Twist> temp_velocities;
            std::vector<geometry_msgs::Accel> temp_accels;
            for (int j = 0; j < min(lookahead, static_cast<int>(drone_pose_list.size()-i)); ++j){
                temp_poses.push_back(drone_pose_list[i+j]);
                temp_velocities.push_back(drone_vel_list[i+j]);
                temp_accels.push_back(drone_accel_list[i+j]);          
            }
            //std::cout << vel_command << std::endl;
            lookahead_states.poses = temp_poses;
            lookahead_states.velocities = temp_velocities;
            lookahead_states.accelerations = temp_accels;
            pub_drone_pos_.publish(lookahead_states);
            ros::Duration(0.1).sleep(); 
            temp_poses.clear();
            temp_velocities.clear();
            temp_accels.clear();
        }
    }

    private: 
        Eigen::Matrix<double, 3, 3> RPY_to_R_matrix(double roll, double pitch, double yaw) {
            Eigen::AngleAxis<double> rollAngle(roll, Eigen::Matrix<double, 1, 3>::UnitX());
            Eigen::AngleAxis<double> pitchAngle(pitch, Eigen::Matrix<double, 1, 3>::UnitY());
            Eigen::AngleAxis<double> yawAngle(yaw, Eigen::Matrix<double, 1, 3>::UnitZ());

            Eigen::Matrix<double, 3, 3> R;

            Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;

            R = q.matrix();

            return (R);
        }

        Eigen::Matrix<double, 3, 3> quat_to_R_matrix(geometry_msgs::Quaternion q) {
            double roll, pitch, yaw;
            tf2::Quaternion quat_tf;
            tf2::fromMsg(q, quat_tf);
            Eigen::Matrix<double, 3, 3> mat_res;
            tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

            return RPY_to_R_matrix(roll, pitch, yaw);
        }

        geometry_msgs::Quaternion RPY_to_quat(double roll, double pitch, double yaw) {
            tf2::Quaternion quaternion_tf2;
            quaternion_tf2.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
            return quaternion;
        }

        void draw_goal_marker(mav_trajectory_generation::Vertex goal){
            Eigen::VectorXd pos;
            goal.getConstraint(mav_trajectory_generation::derivative_order::POSITION,&pos);
            visualization_msgs::Marker marker_aux;
            marker_aux.header.frame_id = "world";
            marker_aux.header.stamp = ros::Time(0);
            marker_aux.id = id_marker;
            id_marker++;
            marker_aux.ns = "point";
            marker_aux.type = visualization_msgs::Marker::CUBE;
            marker_aux.pose.position.x = pos(0);
            marker_aux.pose.position.y = pos(1);
            marker_aux.pose.position.z = pos(2);
            marker_aux.pose.orientation.x = 0;
            marker_aux.pose.orientation.y = 0;
            marker_aux.pose.orientation.z = 0;
            marker_aux.pose.orientation.w = 1;
            marker_aux.scale.x = 0.1;
            marker_aux.scale.y = 0.1;
            marker_aux.scale.z = 0.1;
            marker_aux.color.r = 1.0f;
            marker_aux.color.g = 0.0f;
            marker_aux.color.b = 0.0f;
            marker_aux.color.a = 1.0;
            marker_aux.lifetime = ros::Duration();
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.push_back(marker_aux);
            pub_traj_markers_.publish(marker_array);
        }

        void draw_trajectory_markers(){
            visualization_msgs::MarkerArray markers;
            int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
            double sampling_time = 0.1;
            mav_msgs::EigenTrajectoryPoint::Vector states;
            double sampling_interval = 0.1;
            bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);   
            for(int i=0; i< states.size(); i++) {
                visualization_msgs::Marker marker_aux;
                marker_aux.header.frame_id = "world";
                //marker_aux.header.stamp = ros::Time::now();
                marker_aux.header.stamp = ros::Time(0);
                marker_aux.id = id_marker;
                id_marker++;
                marker_aux.ns = "point";
                marker_aux.type = visualization_msgs::Marker::CUBE;
                marker_aux.pose.position.x = states[i].position_W[0] ;
                marker_aux.pose.position.y = states[i].position_W[1] ;
                marker_aux.pose.position.z = states[i].position_W[2] ;
                marker_aux.pose.orientation.x = 0;
                marker_aux.pose.orientation.y = 0;
                marker_aux.pose.orientation.z = 0;
                marker_aux.pose.orientation.w = 1;
                marker_aux.scale.x = 0.03;
                marker_aux.scale.y = 0.03;
                marker_aux.scale.z = 0.03;
                marker_aux.color.r = 0.925490196f;
                marker_aux.color.g = 0.039215686f;
                marker_aux.color.b = 0.509803922f;
                marker_aux.color.a = 1.0;
                marker_aux.lifetime = ros::Duration();
                markers.markers.push_back(marker_aux);
            }
            pub_traj_markers_.publish(markers);
        }

        void draw_gate_markers(geometry_msgs::Pose gate){
            visualization_msgs::Marker marker_right;
            visualization_msgs::Marker marker_left;
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker line_marker;
            //std::vector<visualization_msgs::Marker> line_marker_vector;
            
            Eigen::Matrix<double, 3, 3> rotate_gate = quat_to_R_matrix(gate.orientation);
            Eigen::Matrix<double, 3, 1> pos_gate(gate.position.x, gate.position.y, gate.position.z);

            marker_right.header.frame_id = "world";  // Change this frame_id according to your setup
            marker_right.header.stamp = ros::Time::now();
            marker_right.type = visualization_msgs::Marker::CUBE;
            marker_right.action = visualization_msgs::Marker::ADD;
            marker_right.ns = "corner_right";
            marker_right.scale.x = 0.2;
            marker_right.scale.y = 0.2;
            marker_right.scale.z = 0.2;
            marker_right.color.a = 1.0;
            marker_right.color.r = 1.0;
            marker_right.color.g = 0.0;
            marker_right.color.b = 0.0;
            marker_right.pose.orientation.w = 1.0;
            marker_right.lifetime = ros::Duration();

            marker_left.header.frame_id = "world";  // Change this frame_id according to your setup
            marker_left.header.stamp = ros::Time::now();
            marker_left.type = visualization_msgs::Marker::SPHERE;
            marker_left.action = visualization_msgs::Marker::ADD;
            marker_left.ns = "corner_left";
            marker_left.scale.x = 0.25;
            marker_left.scale.y = 0.25;
            marker_left.scale.z = 0.25;
            marker_left.color.a = 1.0;
            marker_left.color.r = 1.0;
            marker_left.color.g = 1.0;
            marker_left.color.b = 0.0;
            marker_left.pose.orientation.w = 1.0;
            marker_left.lifetime = ros::Duration();

            line_marker.header.frame_id = "world";  // Change this frame_id according to your setup
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "line";
            line_marker.id = id_marker;
            id_marker++;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.05;  // Line width
            line_marker.pose.orientation.w = 1.0;
            line_marker.lifetime = ros::Duration();

            // Set the color (green in this case)
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;

            float gate_size = 0.75;

            //Generate the gate corners and edges
            Eigen::Matrix<double, 3, 1> move_gate;
            move_gate << 0.0, gate_size, gate_size;
            Eigen::Matrix<double, 3, 1> position2 = pos_gate + rotate_gate * move_gate;
            marker_left.pose.position.x = position2(0);
            marker_left.pose.position.y = position2(1);
            marker_left.pose.position.z = position2(2);
            marker_left.id = id_marker;
            id_marker++;
            line_marker.points.push_back(marker_left.pose.position);
            marker_array.markers.push_back(marker_left);

            move_gate << 0.0, -gate_size, gate_size;
            Eigen::Matrix<double, 3, 1> position = pos_gate + rotate_gate * move_gate;
            marker_right.pose.position.x = position(0);
            marker_right.pose.position.y = position(1);
            marker_right.pose.position.z = position(2);
            marker_right.id = id_marker;
            id_marker++;
            line_marker.points.push_back(marker_right.pose.position);
            marker_array.markers.push_back(marker_right);

            move_gate << 0.0, -gate_size, -gate_size;
            position = pos_gate + rotate_gate * move_gate;
            marker_right.pose.position.x = position(0);
            marker_right.pose.position.y = position(1);
            marker_right.pose.position.z = position(2);
            marker_right.id = id_marker;
            id_marker++;
            line_marker.points.push_back(marker_right.pose.position);
            marker_array.markers.push_back(marker_right);

            move_gate << 0.0, gate_size, -gate_size;
            position = pos_gate + rotate_gate * move_gate;
            marker_left.pose.position.x = position(0);
            marker_left.pose.position.y = position(1);
            marker_left.pose.position.z = position(2);

            marker_left.id = id_marker;
            id_marker++;
            marker_array.markers.push_back(marker_left);
            line_marker.points.push_back(marker_left.pose.position);

            marker_left.pose.position.x = position2(0);
            marker_left.pose.position.y = position2(1);
            marker_left.pose.position.z = position2(2);
            line_marker.points.push_back(marker_left.pose.position);
            marker_array.markers.push_back(line_marker);
            pub_gate_markers_.publish(marker_array);
        }

};

int main(int argc, char** argv) {


	ros::init(argc, argv, "move_drone");
	ros::NodeHandle nh("~");

    // Load the gates
    drone_race race;
    string filegates;
    std::string packagePath = ros::package::getPath("arob_mpc");
    filegates.assign(packagePath+"/src/");
    if (argc>1){
        filegates.append(argv[argc-1]);
    }
    else {
        filegates.append("gates.txt");
    }
    race.readGates(filegates);

    // reset the global it
    race.resetGlobalIt();
    
    ros::Rate loop_rate(10);
    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
        loop_rate.sleep();

        race.incrementGlobalIt();
    }
    race.drawGates();

    //race.generate_trajectory_example();
    race.generate_trajectory();

    while (ros::ok())
    {
        // race.send_command();
        ros::spinOnce();
        loop_rate.sleep();
    }
	return 0;
}
