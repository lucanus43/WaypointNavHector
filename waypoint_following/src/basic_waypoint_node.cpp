#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
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
ROS_INFO("Current position: [%f,%f,%f]", current_pos->pose.position.x, current_pos->pose.position.y, current_pos->pose.position.z);
ROS_INFO("Current goal: [%f,%f,%f]", current_goal.pose.position.x, current_goal.pose.position.y, current_goal.pose.position.z);
ROS_INFO("fabs( current_pos->pose.orientation.x - current_goal.pose.orientation.x ): %f", fabs( current_pos->pose.orientation.x - current_goal.pose.orientation.x ));

// TODO	Force the pose of the UAV to converge to desired pose before moving to next WP.
// ..


//If the current position is close to the current goal for X, Y, & Z
	if( fabs( current_pos->pose.position.x - current_goal.pose.position.x ) < wp_radius && fabs( current_pos->pose.orientation.x - current_goal.pose.orientation.x ) < wp_radius) {
		if( fabs( current_pos->pose.position.y - current_goal.pose.position.y  < wp_radius ) && fabs( current_pos->pose.orientation.y - current_goal.pose.orientation.y ) < wp_radius) {
			if( fabs( current_pos->pose.position.z - current_goal.pose.position.z )  < wp_radius  && fabs( current_pos->pose.orientation.z - current_goal.pose.orientation.z ) < wp_radius) {
				//If there are more waypoints
				wp_counter++;	//Move to the next waypoint
				if( wp_counter < waypoints.size() ) {
					current_goal.pose = waypoints.at(wp_counter);
				} else {
					quit_loop = true;
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
	tmp_wp.position.x = 1.0;	//[1.0, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 3
	tmp_wp.position.x = 2.0;	//[2.0, 0.0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 4
	/*tmp_wp.position.x = -1.0;	//[-1, -5, 50]
	tmp_wp.position.y = -1.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 5
	tmp_wp.position.x = -1.0;	//[0, 0, 50]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
	*/
	
}

int main(int argc, char **argv) {
	//Setup node (must be called before creating variables)
	ros::init( argc, argv, "basic_waypoint" );
	ros::NodeHandle nh;
	ros::Rate loop_rate( 10 );	


	// Local variables
	geometry_msgs::PoseStamped HBII0;		// Initial pose in inertial coords. Set to zero. 	
	hector_uav_msgs::LandingGoal landGoal;	
	hector_uav_msgs::PoseGoal poseGoal;		// PoseGoal object for simpleactionclient PoseActionClient
	ros::Subscriber pos_sub;
	hector_uav_msgs::TakeoffGoal goal;		// Goal (empty message) for TakeoffClient



	//Publishers & Subscribers
	// Publish to /position_goal, and Listen to /ground_truth_to_tf/pose (current pose)
	pos_sub = nh.subscribe( "/ground_truth_to_tf/pose", 1000, position_cb );
	// Generate the waypoints
	generate_waypoints();
		
	//Format output message
	current_goal.header.frame_id = "world";
	current_goal.pose = waypoints.at(wp_counter); //initialize with wp 0
	

	//Write something so we know the node is running
	ROS_INFO( "Publishing position goal..." );


	// Initialise the PoseActionClient
	PoseActionClient poc(nh, "action/pose");
	poc.waitForServer();
	ROS_INFO("Pose client initialised.");
	
	// Initialise the TakeoffClient
	TakeoffClient toc(nh, "action/takeoff");
	toc.waitForServer();
	ROS_INFO("Takeoff client initialised.");
	
	
	// Send take-off goal to toc
	toc.sendGoal(goal);
	
	// Main while loop
	while ( ros::ok() && !quit_loop ) {
		//Update our message so the receiving node knows it is recent
		current_goal.header.stamp = ros::Time::now();
		current_goal.header.seq++;
		current_goal.header.frame_id = "world";

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
	
	// DEBUG: Do not send goal
	//lnc.sendGoal(landGoal);

	ros::shutdown();

	return 0;
}
