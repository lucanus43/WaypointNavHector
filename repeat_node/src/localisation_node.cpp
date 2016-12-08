/* localisation_node.cpp 
   
Desc:	Performs the repeat node for visual teach and repeat implementation.
		Note: BG. - Background tasks
		

Author: Jeffrey Devaraj
Date: 161125 

*/

// ------------------------- DECLARATIONS ------------------------------ //

// C/C++ Includes
#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
// ROS Includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include "actionlib/client/simple_action_client.h"
// OpenCV Includes
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
// Hector includes
#include "hector_uav_msgs/TakeoffAction.h"
#include "hector_uav_msgs/LandingAction.h"
#include "hector_uav_msgs/PoseAction.h"
// Geometry Msgs
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
// TF includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"	// To convert between geometry_msgs and TF libraries
#include <rtabmap_ros/ResetPose.h>
// JDev Files
#include "cvUtility.h"
#include "map_packaging.h"

// ---------------------------- CALLBACK FUNCTIONS ---------------------------//



// -------------------------------------------------------------
/*
main -	Main execution loop

Author: JDev 161125
	
*/
// -------------------------------------------------------------
int main(int argc, char **argv){
	// Initialise ROS
	ros::init(argc, argv, "repeat_node_localisation");
	// Create node handle
	ros::NodeHandle nh;

	// Subscribers and publishers
	ros::Subscriber cloudSub;	// Point cloud subscriber
	ros::Publisher localisationUpdate;	// Publisher to send update from localisation node
	
	// Initialisation
	localisationUpdate = nh.advertise<geometry_msgs::PoseStamped>("repeat_node/locPose", 1);
	cloudSub = nh.subscribe("rtabmap/cloud_map", 100, cloudCallBack);	// from RTABMAP
	submapCloudSub = nh.subscribe("repeat_node/submapCloud",1, submapCloudCallBack);	// From repeat pass
	cloudAlignEstSub = nh.subscribe("repeat_node/cloudAlignment",1, cloudAlignmentCallBack);	// SROR, QRO
	
	// main loop
	while (nh.ok()){
		ros::spinOnce();
		performICP();
	}
	
	
}
