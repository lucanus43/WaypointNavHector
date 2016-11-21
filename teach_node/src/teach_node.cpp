/* teach_node.cpp 
   
Desc:	Performs the teach node for visual teach and repeat implementation.
		Note: BG. - Background tasks

Author: Jeffrey Devaraj
Date: 161114 

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


// Main variables
	bool quitTeachNode = false;

	tf::StampedTransform tfTBL;
	tf::StampedTransform tfTCB;

// Map packaging variables
	bool saveMap = false;

// Odometry variables
	bool firstOdom = true;
	vector<Mat> vecSOCL;
	vector<Mat> vecTCL;
	Mat SOCL = Mat::zeros(3,1,CV_64F);
	Mat TCL = Mat::zeros(3,3,CV_64F);
	Mat TBL;
	Mat TOBhat;
	Mat TCB;
	Mat TOChat;
	Mat TOLhat;
	Mat TCLhat;
	Mat QBL;
	Mat QCB;
// ------------------------- CODE ------------------------------ //



// -------------------------------------------------------------
/*
cloudCallBack - Callback function for pointcloud subscriber
				Derived from: http://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

Author: JDev 161108
	
*/
// -------------------------------------------------------------
void cloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
	
	// Local Variables
	pcl::PCLPointCloud2 pcl_pc2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>); // temp_cloud is in PointXYZ form
    
    // If saveMap is true, then save map to file and set saveMap = false
    if (saveMap){
    	pcl_conversions::toPCL(*input,pcl_pc2);
    	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    	
    	// Update file naming
    	// ..
    	
    	// Save map here.
    	//..
    	
    	// Save poses here.
    	// Call packageMap and package submap
		packageMap(vecSOCL, vecTCL);
    	
    	// Set saveMap to false
    	saveMap = false;
    }
}


// -------------------------------------------------------------
/*
odomCallBack- 	Called when odometry is received.
		
		NOTE: 	/odom does not output the same variables as RTABMAPVIZ (file -> save poses)
				/odom -> SCOC where O is the camera_odometry frame (origin at C, aligned with C)
				RTABMAPVIZ -> SFCK where F is a frame that statically transforms
				to L frame, (QFL constant). K is a coordinate system associated with F.

Author: JDev 161115
	
*/
// -------------------------------------------------------------
void odomCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg){
	// Local variables
	Mat SOCC;
	Mat QCO;
	tf::Quaternion tfQBL;
	tf::Quaternion tfQCB;
	geometry_msgs::Quaternion gmQBL;
	geometry_msgs::Quaternion gmQCB;

	// Initialisation - perform on first callback
	if (firstOdom){
		// Convert tfTBL to tf::Quaternion, then geometry_msgs::Quaternion, then back to cv::Mat TBL
		tfQBL = tfTBL.getRotation();
		tf::quaternionTFToMsg(tfQBL, gmQBL);
		QBL.push_back(gmQBL.w); QBL.push_back(gmQBL.x);
		QBL.push_back(gmQBL.y); QBL.push_back(gmQBL.z);
		TBL = quat2dcm(QBL);
		
		// Obtain TCB from tfTCB
		tfQCB = tfTCB.getRotation();
		tf::quaternionTFToMsg(tfQCB, gmQCB);
		QCB.push_back(gmQCB.w); QCB.push_back(gmQCB.x);
		QCB.push_back(gmQCB.y); QCB.push_back(gmQCB.z);
		TCB = quat2dcm(QCB);
		
		firstOdom = false;
	}
	
	// Process
	// Obtain SOCO
	SOCChat.push_back(odomMsg->pose.pose.position.x);
	SOCChat.push_back(odomMsg->pose.pose.position.y);
	SOCChat.push_back(odomMsg->pose.pose.position.z);
	
	// Notify user that CB functon has been entered
	ROS_INFO("[teach_node] SOCC: [%f,%f,%f]", SOCChat.at<double>(0), SOCChat.at<double>(1), SOCChat.at<double>(2));
	
	// Get TCO from quaternion in odomMsg
	QCO.push_back(odomMsg->pose.pose.orientation.x); 
	QCO.push_back(odomMsg->pose.pose.orientation.y); 
	QCO.push_back(odomMsg->pose.pose.orientation.z); 
	QCO.push_back(odomMsg->pose.pose.orientation.w); 
	
	// Output Quaternion for debugging
	//ROS_INFO("[teach_node] QCO: [%f,%f,%f,%f]", QCO.at<double>(0), QCO.at<double>(1), QCO.at<double>(2), QCO.at<double>(3));
	
	// Have TCB (const), TBL (from TF), TCO (from VO ^), TOL (from TCB, TBL), TBL
	TOChat = quat2dcm(QCO).t();
	// Obtain TCLhat (estimated TCL from VO)
	TOBhat = TOChat*TCB;			// This is NOT constant
	TOLhat = TOBhat*TBL;			// This should be constant since O does not move wrt T
	TCLhat = TOBhat.t()*TCB;		// TCL = TBO*TCB -> From VO
	
	TCL = TCB*TBL; 					// Truth
	
	// TODO: Convert QCM to a tf::quaternion and set TCL as a tf::matrix3x3
		// TCL = tf::matrix3x3(odomMsg->pose.pose.orientation)?
	

	
	// Transform SOCO from O frame to L frame
	SOCLhat = TCLhat.t()*SOCC;
	
	// Output SOCL to WS
	ROS_INFO("[teach_node] SOCL: [%f,%f,%f]", SOCL.at<double>(0), SOCL.at<double>(1), SOCL.at<double>(2));
	
	// Push poses back into vectors
	vecSOCL.push_back(SOCL);
	vecTCL.push_back(TCL);
}


// -------------------------------------------------------------
/*
landingCallBack- 	Called when landing action is called.
					Quits main loop.
			TODO: Use standard callback for generic message
				landingCallBack(const std_msgs::Empty msg)

Author: JDev 161109
	
*/
// -------------------------------------------------------------
void landingCallBack(const hector_uav_msgs::LandingActionGoalConstPtr& landingPose){
	// Set quitImageTransport to true
	quitTeachNode = true;
	// Destroy view
	ROS_INFO("[teach_node] Waypoints complete. Shutting down.");
	// Exit
	ros::shutdown();
}


// -------------------------------------------------------------
/*
main -	Main execution loop

Author: JDev 161108
	
*/
// -------------------------------------------------------------
int main(int argc, char **argv){
	// Initialise ROS
	ros::init(argc, argv, "teach_node");
	// Create node handle
	ros::NodeHandle nh;

	// Local ROS elements (subscribers, listeners, publishers, etc.)
	ros::Subscriber cloudSub;	// Point cloud subscriber
	ros::Subscriber landSub;	// Landing subscriber
	ros::Subscriber odomSub;	// Visual Odometry (VO) subscriber
	tf::TransformListener TBLlistener;
	tf::TransformListener TCBlistener;
	
	// Local variables
	Mat oldSOCL = Mat::zeros(3,1,CV_64F);
	
	
	ros::ServiceClient resetMapClient = nh.serviceClient<std_srvs::Empty>("trigger_new_map");
	std_srvs::Empty::Request resetMapReq;
	std_srvs::Empty::Response resetMapResp;
	
	// Process:
	
	// Initialisation:
	// Subscribe to landing action topic
	landSub = nh.subscribe("action/landing/goal", 1000, landingCallBack);
	// Subscribe to odom topic
	odomSub = nh.subscribe("/rtabmap/odom", 1000, odomCallBack);
	// Subscribe to cloud_map topic
	cloudSub = nh.subscribe("/cloud_map", 1000, cloudCallBack);
	
	// Obtain initial SOCL from VO (will be 0) -> oldSOCL
	
	// Loop (till waypoints are complete/landing initiated)
	while(ros::ok() && !quitTeachNode){
		// BG. Perform Waypoint navigation. (Build a launch file that launches waypointnav) //
		// BG. Rtabmap point cloud building in background (rosspawn? Need VO) //
		
		// BG. Obtain TCL from VO(RTABMAP publishes VO to /odom topic) //
		// BG. Obtain SOCL from VO	//
		
		// TODO: Waypoint navigation in teach node.
		
		// Listen for TBL from TF
		try {
			TBLlistener.lookupTransform("/base_link", "/world", ros::Time(0), tfTBL);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
     		ros::Duration(1.0).sleep();
      		continue;
		}
		
		// Listen for TCB from TF
		try {
			TCBlistener.lookupTransform("/camera_link", "/base_link", ros::Time(0), tfTCB);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
     		ros::Duration(1.0).sleep();
      		continue;
		}
		
		// Check to see if we need a new map
		if (norm(oldSOCL - SOCL) > 2.0){		// |oldSOCL - SOCL| > 2 m
			// Save current point cloud map from cloud_map topic to pcd file
			// if listening to cloud_map topic, set a flag for cb function to save.
			saveMap = true;
			// Reset vecSOCL and vecTCL
			vecSOCL.clear();
			vecTCL.clear();
			// Set RTABMAP to start new submap (service: trigger_new_map (std_srvs/Empty) )
			if (!resetMapClient.call(resetMapReq, resetMapResp)){
				ROS_INFO("[teach_node] Failed to reset map client.");
			}
			oldSOCL = SOCL;
		}// endif
		ros::spin();
	}// end loop
}






