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

#include <vector>

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
ROS_INFO("abs( current_pos->pose.position.y - current_goal.pose.position.y ): %f", fabs( current_pos->pose.position.y -current_goal.pose.position.y ));	

//If the current position is close to the current goal for X, Y, & Z
	if( fabs( current_pos->pose.position.x - current_goal.pose.position.x ) < wp_radius ) {
		if( fabs( current_pos->pose.position.y - current_goal.pose.position.y ) < wp_radius ) {
			if( fabs( current_pos->pose.position.z - current_goal.pose.position.z ) < wp_radius ) {
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
	geometry_msgs::Pose tmp_wp;
	//Waypoint 1
	tmp_wp.orientation.w = 1.0;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	tmp_wp.position.z = 5.0;	//First waypoint is at [0, 0, 5]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 2
	tmp_wp.position.x = 1.0;	//[1, 0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 3
	tmp_wp.position.y = 1.0;	//[1, 1.0, 5.0]
	waypoints.push_back(tmp_wp);
	
	//Waypoint 4
	tmp_wp.position.x = -1.0;	//[-1, -5, 50]
	tmp_wp.position.y = -1.0;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 5
	tmp_wp.position.x = -1.0;	//[0, 0, 50]
	tmp_wp.position.y = 0.0;
	waypoints.push_back(tmp_wp);
}

int main(int argc, char **argv) {
	// Local variables
	geometry_msgs::PoseStamped HBII0;		// Initial pose in inertial coords. Set to zero. 	
	hector_uav_msgs::LandingGoal landGoal;	
	hector_uav_msgs::PoseGoal poseGoal;		// PoseGoal object for simpleactionclient PoseActionClient

	//Setup node
	ros::init( argc, argv, "basic_waypoint" );
	ros::NodeHandle nh;
	ros::Rate loop_rate( 10 );

	//Publishers & Subscribers
	// Publish to /position_goal, and Listen to /position_goal
	ros::Subscriber pos_sub = nh.subscribe( "/ground_truth_to_tf/pose", 1000, position_cb );
	//ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>( "command/pose", 1000 );
	PoseActionClient poc(nh, "action/pose");
	poc.waitForServer();
	ROS_INFO("Pose client initialised.");

	// Generate the waypoints
	generate_waypoints();
		
	//Format output message
	current_goal.header.frame_id = "world";
	current_goal.pose = waypoints.at(wp_counter); //initialize with wp 0
	

	//Write something so we know the node is running
	ROS_INFO( "Publishing position goal..." );


	// Send take-off command to UAV
	TakeoffClient toc(nh, "action/takeoff");
	LandingClient lnc(nh, "action/landing");
	ROS_INFO("Initialised client waiting for connection");
	toc.waitForServer();
	ROS_INFO("Waited for server.");
	
	hector_uav_msgs::TakeoffGoal goal;
	toc.sendGoal(goal);
	
	
	while ( ros::ok() && !quit_loop ) {
		//Update our message so the receiving node knows it is recent
		current_goal.header.stamp = ros::Time::now();
		current_goal.header.seq++;
		current_goal.header.frame_id = "world";

		//Publish messages
		//pos_pub.publish( current_goal );
		// Send current goal to pose
		poseGoal.target_pose = current_goal; 
		poc.sendGoal(poseGoal);

		//Update subscribers and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	// Land UAVadvertise
	// Set initial inertial pose
	ROS_INFO("Initiating landing.");
	HBII0.header.stamp = ros::Time::now();
	HBII0.header.seq++;
	HBII0.header.frame_id = "world";
	// Set target pose
	landGoal.landing_zone = HBII0;
	// Send land goal
	lnc.waitForServer();
	lnc.sendGoal(landGoal);

	ros::shutdown();

	return 0;
}
