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
// JDev Files
#include "map_packaging.h"
#include "cvUtility.h"

using namespace cv;

// ---------------------- GLOBAL VARIABLES -----------------------------//
bool quitTeachNode = false;
bool saveMap = false;
vector<Mat> vecSCMM;
vector<Mat> vecTCM;
Mat SCMM = Mat::zeros(3,1,CV_64F);
Mat TCM = Mat::zeros(3,3,CV_64F);

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
		packageMap(vecSCMM, vecTCM);
    	
    	// Set saveMap to false
    	saveMap = false;
    }
}


// -------------------------------------------------------------
/*
odomCallBack- 	Called when odometry is received.

Author: JDev 161115
	
*/
// -------------------------------------------------------------
void odomCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg){
	// Local variables
	Mat QCM = Mat::zeros(4,1,CV_64F);
	
	// Notify user that CB functon has been entered
	ROS_INFO("[teach_node] Odometry received.");
	
	// Process
	// Obtain SCMM
	SCMM.push_back(odomMsg->pose.pose.position.x);
	SCMM.push_back(odomMsg->pose.pose.position.y);
	SCMM.push_back(odomMsg->pose.pose.position.z);
	// Get TCM from quaternion in odomMsg
	QCM.push_back(odomMsg->pose.pose.orientation.w); QCM.push_back(odomMsg->pose.pose.orientation.x); 
	QCM.push_back(odomMsg->pose.pose.orientation.y); QCM.push_back(odomMsg->pose.pose.orientation.z); 
	TCM = ang2dcm(quat2eul(QCM));
	
	// Push poses back into vectors
	vecSCMM.push_back(SCMM);
	vecTCM.push_back(TCM);
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

	// Local variables
	ros::Subscriber cloudSub;	// Point cloud subscriber
	ros::Subscriber landSub;	// Landing subscriber
	ros::Subscriber odomSub;	// Visual Odometry (VO) subscriber
	Mat oldSCMM = Mat::zeros(3,1,CV_64F);
	
	
	ros::ServiceClient resetMapClient = nh.serviceClient<std_srvs::Empty>("trigger_new_map");
	std_srvs::Empty::Request resetMapReq;
	std_srvs::Empty::Response resetMapResp;
	
	// Process:
	
	// Initialisation:
	// Subscribe to landing action topic
	landSub = nh.subscribe("action/landing/goal", 1000, landingCallBack);
	// Subscribe to odom topic
	odomSub = nh.subscribe("/odom", 1000, odomCallBack);
	// Subscribe to cloud_map topic
	cloudSub = nh.subscribe("/cloud_map", 1000, cloudCallBack);
	
	// Obtain initial SCMM from VO (will be 0) -> oldSCMM
	
	// Loop (till waypoints are complete/landing initiated)
	while(ros::ok() && !quitTeachNode){
		// BG. Perform Waypoint navigation. (Build a launch file that launches waypointnav) //
		// BG. Rtabmap point cloud building in background (rosspawn? Need VO) //
		
		// BG. Obtain TCM from VO(RTABMAP publishes VO to /odom topic) //
		// BG. Obtain SCMM from VO	//
		
		if (norm(oldSCMM - SCMM) > 2.0){		// |oldSCMM - SCMM| > 2 m
			// Save current point cloud map from cloud_map topic to pcd file
			// if listening to cloud_map topic, set a flag for cb function to save.
			saveMap = true;
			// Reset vecSCMM and vecTCM
			vecSCMM.clear();
			vecTCM.clear();
			// Set RTABMAP to start new submap (service: trigger_new_map (std_srvs/Empty) )
			if (!resetMapClient.call(resetMapReq, resetMapResp)){
				ROS_INFO("[teach_node] Failed to reset map client.");
			}
			oldSCMM = SCMM;
		}// endif
	}// end loop
}






