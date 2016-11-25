/* repeat_node.cpp 
   
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
#include "hector_uav_msgs/LandingAction.h"
// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
// TF includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf/transform_datatypes.h>
#include "tf/LinearMath/Transform.h"	// To convert between geometry_msgs and TF libraries
// JDev Files
#include "cvUtility.h"
#include "map_packaging.h"


using namespace cv;

// ---------------------- GLOBAL VARIABLES -----------------------------//





// -------------------------------------------------------------
/*
cloudCallBack - Callback function for pointcloud subscriber
				Derived from: http://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

Author: JDev 161125
	
*/
// -------------------------------------------------------------
void cloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
	// Local Variables
	pcl::PCLPointCloud2 pcl_pc2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>); // temp_cloud is in PointXYZ form
	
	// Process
	pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    // Perform ICP on temp_cloud
    // ..
    
    // Obtain error displacement and transformation matrix
    // SRO = error displacement, TRO = error transform
    // TODO: Work out what coordinates SRO is in.
    
    
}



// -------------------------------------------------------------
/*
odomCallBack- 	Called when odometry is received.
				R: Repeat pass odometry frame
		

Author: JDev 161125
	
*/
// -------------------------------------------------------------

void odomCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg){
	// Local variables
	Mat SCRChat = Mat::zeros(3,1,CV_64F);
	Mat QCRhat = Mat::zeros(3,1,CV_64F);
	
	// Process
	// Obtain SCRC
	SCRChat.at<double>(0) = (odomMsg->pose.pose.position.x);
	SCRChat.at<double>(1) = (odomMsg->pose.pose.position.y);
	SCRChat.at<double>(2) = (odomMsg->pose.pose.position.z);
	
	// Get QCR from quaternion in odomMsg
	QCRhat.push_back(odomMsg->pose.pose.orientation.x); 
	QCRhat.push_back(odomMsg->pose.pose.orientation.y); 
	QCRhat.push_back(odomMsg->pose.pose.orientation.z); 
	QCRhat.push_back(odomMsg->pose.pose.orientation.w); 
	
	// Convert QCR to TCR
	TCRhat = quat2dcm(QCRhat);
	
	// Compute SRLLhat and TRLhat - needs to only be done once per submap - 
	// TODO: Initialise TCLhat and SCLLhat - GPS/Maggrav update to get SCLL/TCL initially?
	TRLhat = TRChat*TCLhat;		
	SRLLhat = -TRLhat*SCRChat + SCLLhat; 	
	
	// 'Truth' SRLL and TRL
	TRL = TRChat*TCL;						// Need TCL
	SRLL = -TRL*SCRChat + SCLL;				// Need SCLL
	
	// Update TCLhat and SCLLhat
	TCLhat = TCRhat*TRLhat;
	SCLLhat = TCLhat.t()*SCRChat + SRLLhat;
}


// -------------------------------------------------------------
/*
positionCallBack- 	Called when SBLL/TBL is received (listening for TBL - truth)
					Waypoint navigation is performed here.

Author: JDev 161125
	
*/
// -------------------------------------------------------------
void positionCallBack( const geometry_msgs::PoseStamped::ConstPtr& SBLLTBL ) {
	// Local variables
	// ..
	
	// Force SCLLhat to converge to SCLL_cmd and TCLhat to converge to TCL_cmd
	if( fabs( SCLLhat.at<double>(0) -  SCLL_cmd.at<double>(0) ) < wp_radius && fabs( QCLhat.at<double>(0) -  QCL_cmd.at<double>(0) ) < wp_radius) {
		if( fabs( SCLLhat.at<double>(1) - SCLL_cmd.at<double>(1))  < wp_radius && fabs( QCLhat.at<double>(1) -  QCL_cmd.at<double>(1) ) < wp_radius) {
			if( fabs( SCLLhat.at<double>(2) - SCLL_cmd.at<double>(2) )  < wp_radius && fabs( QCLhat.at<double>(2) -  QCL_cmd.at<double>(2) ) < wp_radius) {
				// ..
				
			}
		}
	}
}

// -------------------------------------------------------------
/*
generateWaypoints -	Generates waypoints from the given poses.txt file

Author: JDev 161125
	
*/
// -------------------------------------------------------------
void generateWaypoints(string poseFileName){
	// Local variables
	// ..
	

	// Read in waypoints from poseFile along with SOLLhat
	// Generate waypoints as SCLL_cmd using SCOLhat and SOLLhat
		// SCLL_cmd = SCOLhat + SOLLhat
	// Subsample pose file
}

// -------------------------------------------------------------
/*
loadSubmapPCD -	Loads a Pointcloud into memory

Author: JDev 161125
	
*/
// -------------------------------------------------------------
void loadSubmapPCD(string PCDFileName){
	// Local variables
	// ..
	
	// Load submap into a pointxyz variable
	// ..
	
}



// -------------------------------------------------------------
/*
main -	Main execution loop

Author: JDev 161125
	
*/
// -------------------------------------------------------------
int main(int argc, char **argv){
	// Initialise ROS
	ros::init(argc, argv, "repeat_node");
	// Create node handle
	ros::NodeHandle nh;

	// Local  ROS elements (subscribers, listeners, publishers, etc.)
	ros::Subscriber odomSub;	// Visual Odometry (VO) subscriber
	ros::Subscriber cloudSub;	// Point cloud subscriber
	
	// Local variables
	// ..
	
	// Initialisation
	// Subscribe to odom topic
	odomSub = nh.subscribe("/rtabmap/odom", 1, odomCallBack);
	// Subscribe to cloud_map topic
	cloudSub = nh.subscribe("rtabmap/cloud_map", 1000, cloudCallBack);
	
	// Start on submap_0 -> pose_0.txt, submap_0.pcd.
	generateWaypoints("pose_0.txt");
	loadSubmapPCD("submap_0.pcd");
	
	
	// Loop
	while(nh.ok()){
		// Waypoint navigation using SCLLhat and SCLL_cmd, TCLhat and TCL_cmd (convert to B frame?)
			
		// if waypoints complete
			// if next submap exists, move to next submap.
			// else exit.
		// endif
	}
	
}






















