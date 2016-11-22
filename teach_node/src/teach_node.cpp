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
	Mat SOCLhat = Mat::zeros(3,1,CV_64F);
	Mat TCL = Mat::zeros(3,3,CV_64F);
	Mat TBL;
	Mat TOBhat;
	Mat TCB;
	Mat TOChat;
	Mat TOLhat;
	Mat TCLhat;
	Mat QBL = Mat::zeros(4,1,CV_64F);
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
    
    // Notify user of entering function
    ROS_INFO("[teach_node] Entered cloudCallBack.");
    
    // If saveMap is true, then save map to file and set saveMap = false
    if (saveMap){
    	pcl_conversions::toPCL(*input,pcl_pc2);
    	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    	
    	// TODO: Update file naming
    	  pcl::io::savePCDFileASCII ("test_pcd.pcd", *temp_cloud);
    	  std::cerr << "Saved " << temp_cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
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
	Mat SOCChat;
	Mat QCOhat;
	tf::Quaternion tfQBL;
	tf::Quaternion tfQCB;
	geometry_msgs::Quaternion gmQBL;
	geometry_msgs::Quaternion gmQCB;
	
	
	// Initialisation - perform on first callback
	if (firstOdom){
		
		//ROS_INFO("Made it here. Quitting.");
		//quitTeachNode = true;
		
		// Obtain TCB (static transform, QCB = [0.000, 0.707, 0.000, 0.707])
		// TODO: Work out why camera_link to base_link is shown in rqt_tf_tree but lookupTransform() fails?
		QCB = Mat::zeros(4,1,CV_64F);
		QCB.at<double>(1) = 0.707; QCB.at<double>(3) = 0.707;
		TCB = quat2dcm(QCB);
		
		firstOdom = false;
	}
	
	// Process
	// Obtain SOCC
	SOCChat.push_back(odomMsg->pose.pose.position.x);
	SOCChat.push_back(odomMsg->pose.pose.position.y);
	SOCChat.push_back(odomMsg->pose.pose.position.z);
	
	// Notify user that CB functon has been entered
	//ROS_INFO("[teach_node] SOCC: [%f,%f,%f]", SOCChat.at<double>(0), SOCChat.at<double>(1), SOCChat.at<double>(2));
	
	// Get TCO from quaternion in odomMsg
	QCOhat.push_back(odomMsg->pose.pose.orientation.x); 
	QCOhat.push_back(odomMsg->pose.pose.orientation.y); 
	QCOhat.push_back(odomMsg->pose.pose.orientation.z); 
	QCOhat.push_back(odomMsg->pose.pose.orientation.w); 
	
	// Output Quaternion for debugging
	//ROS_INFO("[teach_node] QCO: [%f,%f,%f,%f]", QCOhat.at<double>(0), QCOhat.at<double>(1), QCOhat.at<double>(2), QCOhat.at<double>(3));
	
	// Have TCB (const), TBL (from TF), TCO (from VO ^), TOL (from TCB, TBL), TBL
	TOChat = quat2dcm(QCOhat).t();
	
	// Get TBL from tf
	// Convert tfTBL to tf::Quaternion, then geometry_msgs::Quaternion, then back to cv::Mat TBL
	tfQBL = tfTBL.getRotation();
	tf::quaternionTFToMsg(tfQBL, gmQBL);
	QBL.at<double>(3) = (gmQBL.w); QBL.at<double>(0) = (gmQBL.x); // QBL [x,y,z,w]
	QBL.at<double>(1) = (gmQBL.y); QBL.at<double>(2) = (gmQBL.z);
	//cout << "QBL: " << QBL << endl;
	TBL = quat2dcm(QBL);			// Truth
	
	// Obtain TCLhat (estimated TCL from VO)
	TOBhat = TOChat*TCB;			// This is NOT constant
	TOLhat = TOBhat*TBL;			// This should be constant since O does not move wrt L
	TCLhat = TOChat.t()*TOLhat;		// TCL = TCO*TOB // TCL = TCO*TOL -> From VO
	
	cout << "TCLhat: " << TCLhat << endl;
	cout << "TCL: " << TCL << endl;
	TCL = TCB*TBL; 					// Truth
	
	//cout << "TOChat: " << TOChat << endl;
	//cout << "TCB: " << TCB << endl;
	//cout << "TOChat*TCB: " << TOChat*TCB << endl;
	//cout << "TOBhat: " << TOBhat << endl;
	//cout << "TOLhat: " << TOLhat << endl;
	//cout << "TBL: " << TBL << endl;

	
	// TODO: Convert QCM to a tf::quaternion and set TCL as a tf::matrix3x3
		// TCL = tf::matrix3x3(odomMsg->pose.pose.orientation)?
	
	// TODO: Get SOLL for map.
	
	// Transform SOCO from O frame to L frame
	SOCLhat = TCLhat.t()*SOCChat;
	
	// Output SOCL to WS
	//ROS_INFO("[teach_node] SOCL: [%f,%f,%f]", SOCLhat.at<double>(0), SOCLhat.at<double>(1), SOCLhat.at<double>(2));
	
	// Push poses back into vectors
	// TODO: Only save poses when a certain distance has been travelled or angular change has been observed.
	// For now only save SOCL and TCL (camera pose wrt. local level frame).
	vecSOCL.push_back(SOCLhat);
	vecTCL.push_back(TCLhat);
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
	//tf::TransformListener TCBlistener;
	
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
	oldSOCL = SOCLhat;
	
	// Notify user of initialisation
	ROS_INFO("[teach_node] teach_node initialised. Entering loop.");
	
	// Loop (till waypoints are complete/landing initiated)
	while(ros::ok() && !quitTeachNode){
		// BG. Perform Waypoint navigation. (Build a launch file that launches waypointnav) //
		// BG. Rtabmap point cloud building in background (rosspawn? Need VO) //
		
		// BG. Obtain TCL from VO(RTABMAP publishes VO to /odom topic) //
		// BG. Obtain SOCL from VO	//
		
		// TODO: Waypoint navigation in teach node.
		
		// Listen for TCB from TF
		/*try {
			TCBlistener.waitForTransform("base_link", "camera_link", ros::Time(0), ros::Duration(0.5));
			TCBlistener.lookupTransform("base_link", "camera_link", ros::Time(0), tfTCB);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
     		ros::Duration(1.0).sleep();
      		continue;
		}*/
		
		// Listen for TBL from TF
		try {
			TBLlistener.waitForTransform("world", "base_link", ros::Time(0), ros::Duration(0.5));
			TBLlistener.lookupTransform("world", "base_link", ros::Time(0), tfTBL);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
     		ros::Duration(1.0).sleep();
      		continue;
		}
	
		
		// Check to see if we need a new map
		if (norm(oldSOCL - SOCLhat) > 2.0){		// |oldSOCL - SOCL| > 2 m
			// Save current point cloud map from cloud_map topic to pcd file
			// if listening to cloud_map topic, set a flag for cb function to save.
			ROS_INFO("[teach_node] Saving map to file.");
			saveMap = true;
			// Reset vecSOCL and vecTCL
			vecSOCL.clear();
			vecTCL.clear();
			// Set RTABMAP to start new submap (service: trigger_new_map (std_srvs/Empty) )
			if (!resetMapClient.call(resetMapReq, resetMapResp)){
				ROS_INFO("[teach_node] Failed to reset map client.");
			}
			oldSOCL = SOCLhat;
		}// endif
		ros::spin();
	}// end loop
}






