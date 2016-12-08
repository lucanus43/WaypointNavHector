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

using namespace cv;

// ---------------------- GLOBAL VARIABLES -----------------------------//

// Main variables
	pcl::PointCloud<pcl::PointXYZ>::Ptr submapCloud (new pcl::PointCloud<pcl::PointXYZ>);
	Mat SRORhat = Mat::zeros(3,1,CV_64F);
	Mat QROhat = Mat::zeros(3,1,CV_64F);
	Mat errTOR = Mat::zeros(3,3,CV_64F);
	Mat errSORO = Mat::zeros(3,1,CV_64F);

// ICP variables
	Mat TORhat_icp = Mat::zeros(3,3,CV_64F);
	Mat SORRhat_icp = Mat::zeros(3,1,CV_64F);
	Mat QRLhat_icp = Mat::zeros(4,1,CV_64F);
	Mat SRLLhat_icp = Mat::zeros(3,1,CV_64F);
	Mat TCLhat_icp = Mat::zeros(3,3,CV_64F);
	Mat SCLLhat_icp = Mat::zeros(3,1,CV_64F);
	Mat QBLhat_icp = Mat::zeros(4,1,CV_64F);
	Mat SBLLhat_icp = Mat::zeros(3,1,CV_64F);
	Mat QCLhat_icp = Mat::zeros(4,1,CV_64F);
	Mat SORLhat_icp = Mat::zeros(3,1,CV_64F);
	pcl::PointCloud<pcl::PointXYZ>::Ptr observedCloud(new pcl::PointCloud<pcl::PointXYZ>); // temp_cloud is in PointXYZ form
	


// ---------------------------- CALLBACK FUNCTIONS ---------------------------//


// -------------------------------------------------------------
/*
cloudAlignmentCallBack - Callback function received transform from
						 R frame to O frame.
				
Author: JDev 161208
	
*/
// -------------------------------------------------------------
void cloudAlignmentCallBack( const geometry_msgs::PoseStamped::ConstPtr& gmRO ){
	// Local variables
	// ..
	
	// Process
	SRORhat.at<double>(0) = gmRO->pose.position.x;
	SRORhat.at<double>(1) = gmRO->pose.position.y;
	SRORhat.at<double>(2) = gmRO->pose.position.z;
	QROhat.at<double>(0) = gmRO->pose.orientation.x;
	QROhat.at<double>(1) = gmRO->pose.orientation.y;
	QROhat.at<double>(2) = gmRO->pose.orientation.z;
	QROhat.at<double>(3) = gmRO->pose.orientation.w;
}

// -------------------------------------------------------------
/*
submapCloudCallBack - Callback function for pointcloud subscriber
				Derived from: http://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

Author: JDev 161208
	
*/
// -------------------------------------------------------------
void submapCloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
	// Local Variables
	pcl::PCLPointCloud2 pcl_pc2;

	// Processing
	pcl_conversions::toPCL(*input,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*submapCloud);
	submapCloud->is_dense = false;
}


// -------------------------------------------------------------
/*
cloudCallBack - Callback function for pointcloud subscriber
				Derived from: http://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

Author: JDev 161208
	
*/
// -------------------------------------------------------------
void cloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
	// Local Variables
	pcl::PCLPointCloud2 pcl_pc2;

	// Processing
	pcl_conversions::toPCL(*input,pcl_pc2);
	pcl::fromPCLPointCloud2(pcl_pc2,*observedCloud);
	observedCloud->is_dense = false;
}


// ----------------------- PROCESS FUNCTIONS ---------------------- //
// -------------------------------------------------------------
/*
performICP -		performs ICP algorithm on observedCloud and submapCloud
					
					NOTE: Called from Main since placing it in callback lags system.

Author: JDev 161208
	
*/
// -------------------------------------------------------------
void performICP(){
	// Local variables
	pcl::PointCloud<pcl::PointXYZ> alignedCloud;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedObsCloud(new pcl::PointCloud<pcl::PointXYZ>); 
	Mat SPOR_obs = Mat::zeros(3,1,CV_64F);
	Mat SPOO_obs = Mat::zeros(3,1,CV_64F);
	
	// Process
	// Set transformedObsCloud to have the same data as observedCloud
	*transformedObsCloud = *observedCloud;
	
	if (doICP){
		// Observed cloud is a series of SPRR_obs points. SubmapCloud is a series of SPOO_sub points
		// Obtain SPOR_obs using
		// SPOR_obs = SPRR_obs + SRORhat where SRORhat = TRLhat*(SRCLhat + SOLLhat)
		// Need SPOO_obs, so use SPOO_obs = TROhat*SPOR_obs (TROhat = TRLhat*TOLhat.t())
		// ICP between SPOO_obs, SPOO_sub will give error (error(SORO), error (TOR)). 
		// Assume error in SPOO_obs and SPOO_sub is equivalent to error in SORO,
		// Correct SRORhat, TROhat.
		
		// Apply SRORhat to observedCloud
		for (size_t i = 0; i < observedCloud->points.size (); ++i){
			SPOR_obs.at<double>(0) = observedCloud->points[i].x + SRORhat.at<double>(0);
			SPOR_obs.at<double>(1) = observedCloud->points[i].y + SRORhat.at<double>(1);
			SPOR_obs.at<double>(2) = observedCloud->points[i].z + SRORhat.at<double>(2);
			
			SPOO_obs = quat2dcm(QROhat).t()*SPOR_obs;
			
    		transformedObsCloud->points[i].x = SPOO_obs.at<double>(0);
			transformedObsCloud->points[i].y = SPOO_obs.at<double>(1);
			transformedObsCloud->points[i].z = SPOO_obs.at<double>(2);
		}
		
		// Perform icp on transformedObsCloud
		icp.setInputSource(transformedObsCloud);
	 	icp.setInputTarget(submapCloud);
	 	
	 	ROS_INFO_STREAM("TROhat: " <<  quat2dcm(QROhat));
	 	icp.align(alignedCloud);

		// Obtain final transformation from ICP
		// This is error in SORO and TRO
		// If fitness score is < 0.1, no need to do ICP anymore.
		if (icp.getFitnessScore() < 0.05){
			doICP = false;
			// Align to alignedCloud
			pcl::io::savePCDFileASCII ("obsCloud.pcd", *observedCloud);
			pcl::io::savePCDFileASCII ("transformedObs.pcd", *transformedObsCloud);
			pcl::io::savePCDFileASCII ("aligned.pcd", alignedCloud);
		
		
			std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  			icp.getFitnessScore() << std::endl;
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++) {
					errTOR.at<double>(i,j) = icp.getFinalTransformation()(i,j);
				}
				errSORO.at<double>(i) = icp.getFinalTransformation()(i,3);
			}
			cout << "errTOR: " << errTOR << endl;
			cout << "errSORO: " << errSORO << endl;
		}
	}
}


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
