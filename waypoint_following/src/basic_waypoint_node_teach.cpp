#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hector_uav_msgs/TakeoffAction.h"
#include "hector_uav_msgs/LandingAction.h"
#include "hector_uav_msgs/PoseAction.h"
#include "actionlib/client/simple_action_client.h"

// TF includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf/transform_datatypes.h>
 #include "tf/LinearMath/Transform.h"	// To convert between geometry_msgs and TF libraries

#include <vector>

// Definitions
const double PI_F = 3.1415926535897931;

// Pose Goal message
geometry_msgs::PoseStamped current_goal;
geometry_msgs::PoseWithCovarianceStamped truePos;
std::vector<geometry_msgs::Pose> waypoints;
int wp_counter = 0;
double wp_radius = 0.01;
bool quit_loop = false;

// Takeoff client (SimpleActionClient from actionlib)
typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseActionClient;

// Callback function for the subscribe() function
void position_cb( const geometry_msgs::PoseStamped::ConstPtr& current_pos ) {
// Set truePos to current_pos
truePos.pose.pose.position.x = current_pos->pose.position.x;
truePos.pose.pose.position.y = current_pos->pose.position.y;
truePos.pose.pose.position.z = current_pos->pose.position.z;
truePos.pose.pose.orientation.x = current_goal.pose.orientation.x;
truePos.pose.pose.orientation.y = current_goal.pose.orientation.y;
truePos.pose.pose.orientation.z = current_goal.pose.orientation.z;
truePos.pose.pose.orientation.w = current_goal.pose.orientation.w;
// Set truePos covariances
truePos.pose.covariance[0] = 0.5;
truePos.pose.covariance[7] = 0.5;
truePos.pose.covariance[14] = 0.5;
truePos.pose.covariance[21] = 0.5;
truePos.pose.covariance[28] = 0.5;
truePos.pose.covariance[35] = 0.5;

	
// Waypoint nav
ROS_INFO("Current position: [%f,%f,%f]", current_pos->pose.position.x, current_pos->pose.position.y, current_pos->pose.position.z);
ROS_INFO("Current goal: [%f,%f,%f]", current_goal.pose.position.x, current_goal.pose.position.y, current_goal.pose.position.z);
ROS_INFO("fabs( current_pos->pose.orientation.x - current_goal.pose.orientation.x ): %f", fabs( current_pos->pose.orientation.x - current_goal.pose.orientation.x ));

// Force the pose of the UAV to converge to desired pose before moving to next WP.
//If the current position is close to the current goal for X, Y, & Z
	if( (fabs( current_pos->pose.position.x - current_goal.pose.position.x ) < wp_radius) && (fabs( current_pos->pose.orientation.x - current_goal.pose.orientation.x ) < wp_radius)) {
		if( (fabs( current_pos->pose.position.y - current_goal.pose.position.y)  < wp_radius ) && (fabs( current_pos->pose.orientation.y - current_goal.pose.orientation.y ) < wp_radius)) {
			if( (fabs( current_pos->pose.position.z - current_goal.pose.position.z )  < wp_radius)  && (fabs( current_pos->pose.orientation.z - current_goal.pose.orientation.z ) < wp_radius)) {
				//If there are more waypoints
				wp_counter++;	//Move to the next waypoint
				if( wp_counter < waypoints.size() ) {
					current_goal.pose = waypoints.at(wp_counter);
				} else {
					//quit_loop = true;
					
					ROS_INFO( "Finished the waypoint path!" );
				}
			}
		}
	}
}

void generate_waypoints() {
	// Local variables
	geometry_msgs::Pose tmp_wp;
	geometry_msgs::Quaternion gm_temp_quat;
	tf::Quaternion tf_temp_quat;

	// Processing
	
	//Waypoint 1 - > YPR = [0,0,0]
	tf_temp_quat.setEulerZYX(PI_F/2, 0.0, 0.0);
	ROS_INFO("Debug output");
	//  Convert TF quaternion to geometry_msgs
	tf::quaternionTFToMsg(tf_temp_quat, gm_temp_quat);
	// Set waypoint orientation
	tmp_wp.orientation = gm_temp_quat;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	// Set waypoint displacement (rel. local-level frame)
	tmp_wp.position.x = -0.1;
	tmp_wp.position.y = -0.1;
	tmp_wp.position.z = 5.0;	//First waypoint is at [0, 0, 5]
	// Push back
	waypoints.push_back(tmp_wp);
	
	//Waypoint 2 - > YPR = [90,0,0]
	
	//tmp_wp.position.x = 1.0;	//[1, 0, 5.0]
	tf_temp_quat.setEulerZYX(PI_F/2, 0.0, 0.0);
	//  Convert TF quaternion to geometry_msgs
	tf::quaternionTFToMsg(tf_temp_quat, gm_temp_quat);
	// Set waypoint orientation
	tmp_wp.orientation = gm_temp_quat;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	// Set waypoint displacement (rel. local-level frame)
	tmp_wp.position.x = 0.4;	//[1.0, 0, 5.0]
	tmp_wp.position.y = 0;
	tmp_wp.position.z = 5.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 3
	tmp_wp.position.x = 2.0;	//[2.0, 0.0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 4
	tmp_wp.position.x = 2.5;	//[3, 0, 5.0]
	tmp_wp.position.y = -0.5;	//[3, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 5
	tmp_wp.position.x = 3.0;	//[0, 0, 5.0]
	tmp_wp.position.y = -1.0;	//[3, 0, 5.0
	waypoints.push_back(tmp_wp);
	
	//Waypoint 6
	tmp_wp.position.x = 3.5;	//[0, 0, 5.0]
	tmp_wp.position.y = -1.5;	//[3, 0, 5.0
	waypoints.push_back(tmp_wp);
	
	//Waypoint 7
	tmp_wp.position.x = 4.0;	//[0, 0, 5.0]
	
	waypoints.push_back(tmp_wp);
	
	//Waypoint 8
	tmp_wp.position.x = 4.5;	//[0, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 9
	tmp_wp.position.x = 5.0;	//[0, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 10
	tmp_wp.position.x = 5.5;	//[0, 0, 5.0]
	//tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 11
	tmp_wp.position.x = 6.0;	//[0, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 12
	tmp_wp.position.x = 6.5;	//[0, 0, 5.0]
	//tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 13
	tmp_wp.position.x = 7.0;	//[0, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 14
	tmp_wp.position.x = 7.5;	//[0, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 15
	tmp_wp.position.x = 8.0;	//[0, 0, 5.0]
	tmp_wp.position.y = -1.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 16
	tmp_wp.position.x = 8.5;	//[0, 0, 5.0]
	tmp_wp.position.y = -0.5;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 17
	tmp_wp.position.x = 9.0;	//[0, 0, 5.0]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 18
	tmp_wp.position.x = 9.5;	//[0, 0, 5.0]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 19
	tmp_wp.position.x = 10.0;	//[0, 0, 5.0]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
	
	//Waypoint 20
	tmp_wp.position.x = 10.5;	//[0, 0, 5.0]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 21
	tmp_wp.position.x = 11.0;	//[0, 0, 5.0]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 22
	tmp_wp.position.x = 11.5;	//[0, 0, 5.0]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	
}

int main(int argc, char **argv) {
	//Setup node (must be called before creating variables)
	ros::init( argc, argv, "basic_waypoint" );
	ros::NodeHandle nh;
	ros::Rate loop_rate( 50 );	


	// Local variables
	geometry_msgs::PoseStamped HBII0;		// Initial pose in inertial coords. Set to zero. 	
	hector_uav_msgs::LandingGoal landGoal;	
	hector_uav_msgs::PoseGoal poseGoal;		// PoseGoal object for simpleactionclient PoseActionClient
	ros::Subscriber pos_sub;
	ros::Publisher pos_pub;					// Publish truth data
	ros::ServiceClient calibrateIMUClient;
	std_srvs::Empty emptySrv;
	hector_uav_msgs::TakeoffGoal goal;		// Goal (empty message) for TakeoffClient



	//Publishers & Subscribers
	// Publish to /position_goal, and Listen to /ground_truth_to_tf/pose (current pose)
	pos_sub = nh.subscribe( "ground_truth_to_tf/pose", 1000, position_cb );
	pos_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("poseupdate", 1);
	// Services
	calibrateIMUClient = nh.serviceClient<std_srvs::Empty>("/raw_imu/calibrate");
	
	// Generate the waypoints
	generate_waypoints();
		
	//Format output message
	current_goal.header.frame_id = "world";
	current_goal.pose = waypoints.at(wp_counter); //initialize with wp 0
	

	//Write something so we know the node is running
	ROS_INFO( "Publishing position goal..." );
	
	// Publish IMU reset request
	/*if(!calibrateIMUClient.call(emptySrv)){
		ROS_INFO_STREAM("Failed to calibrate IMU");
	} else {
		ROS_INFO("[repeat_node] Succeeded in calibrating IMU.");
	}*/


	// Initialise the PoseActionClient
	PoseActionClient poc(nh, "action/pose");
	poc.waitForServer();
	ROS_INFO("Pose client initialised.");
	
	// Initialise the TakeoffClient
	TakeoffClient toc(nh, "action/takeoff");
	toc.waitForServer();
	ROS_INFO("Takeoff client initialised.");
	
	// TODO: Initialise with true position
	/*while(init){
		ros::spinOnce();
		// truePos header
		truePos.header.stamp = ros::Time::now();
		truePos.header.seq++;
		truePos.header.frame_id = "world";	
		
		// Broadcast truepos as a pose update (for repeat node)
		pos_pub.publish(truePos);
	}*/
	// Spin once to get estimated pose updated to true pose.
	ros::spinOnce();
	
	
	// Send take-off goal to toc
	toc.sendGoal(goal);
	
	// Main while loop
	while ( ros::ok() && !quit_loop ) {
		//Update our message so the receiving node knows it is recent
		current_goal.header.stamp = ros::Time::now();
		current_goal.header.seq++;
		current_goal.header.frame_id = "world";
		
		// truePos header
		truePos.header.stamp = ros::Time::now();
		truePos.header.seq++;
		truePos.header.frame_id = "world";	
		
		// Broadcast truepos as a pose update (for repeat node)
		pos_pub.publish(truePos);

		// Send current goal to pose
		poseGoal.target_pose = current_goal; 
		poc.sendGoal(poseGoal);

		//Update subscribers and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	// Land UAVadvertise
	// Initialise landing client
	LandingClient lnc(nh, "action/landing");
	lnc.waitForServer();
	ROS_INFO("Landing client initialised.");
	// Set initial inertial pose
	HBII0.header.stamp = ros::Time::now();
	HBII0.header.seq++;
	HBII0.header.frame_id = "world";
	// Set target pose
	landGoal.landing_zone = HBII0;
	// Send land goal
	lnc.sendGoal(landGoal);

	ros::shutdown();

	return 0;
}