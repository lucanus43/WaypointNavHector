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
	Mat errQRO = Mat::zeros(4,1,CV_64F);
	Mat QROhat = Mat::zeros(4,1,CV_64F);
	Mat TReR = Mat::zeros(3,3,CV_64F);
	Mat SReRR = Mat::zeros(3,1,CV_64F);
	Mat errTRO = Mat::zeros(3,3,CV_64F);
	Mat errSROO = Mat::zeros(3,1,CV_64F);
	Mat SOROhat = Mat::zeros(3,1,CV_64F);
	geometry_msgs::PoseStamped gmerrRO;
	bool submapCloudAvailable = false;
	bool observedCloudAvailable = false;
	bool poseAvailable = false;
	std::string map_location;
	bool publishLoc = false;
	
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
	poseAvailable = true;
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
	submapCloudAvailable = true;
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
	observedCloudAvailable = true;
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSubCloud(new pcl::PointCloud<pcl::PointXYZ>); 
	Mat SPRO_sub = Mat::zeros(3,1,CV_64F);
	Mat SPRR_sub = Mat::zeros(3,1,CV_64F);
	//Mat SPOR_obs = Mat::zeros(3,1,CV_64F);
	//Mat SPOO_obs = Mat::zeros(3,1,CV_64F);
	
	// Process
	// Set transformedSubCloud to have the same data as submapCloud
	*transformedSubCloud = *submapCloud;
	

	
	if (observedCloudAvailable && poseAvailable){
		// Set poseAvailable, submapCloudAvailable and cloudAvailable to false
		poseAvailable = false;
		observedCloudAvailable = false;
		submapCloudAvailable = false;
	
		// Need to determine errSORO/errTOR
		// Observed cloud is a series of SPRR_obs points. SubmapCloud is a series of SPOO_sub points
		// Obtain SPOR_obs using
		// SPRO_sub = SPOO_sub + SOROhat 
		// Need SPRR_sub, so use SPRR_sub = TROhat*SPRO_sub
		// ICP between SPRR_obs, SPRR_sub will give error (error(SRReR), error (TReR)). 
		// DO NOT ASSUME error in SPOO_obs and SPOO_sub is equivalent to error in SORO,
		// Correct SRORhat, TROhat.
		
		// Apply SOROhat to submapCloud
		SOROhat = -quat2dcm(QROhat).t()*SRORhat;
		for (size_t i = 0; i < submapCloud->points.size (); ++i){
			SPRO_sub.at<double>(0) = submapCloud->points[i].x + SOROhat.at<double>(0);
			SPRO_sub.at<double>(1) = submapCloud->points[i].y + SOROhat.at<double>(1);
			SPRO_sub.at<double>(2) = submapCloud->points[i].z + SOROhat.at<double>(2);
			
		//	SPOR_obs.at<double>(0) = observedCloud->points[i].x + SRORhat.at<double>(0);
		//	SPOR_obs.at<double>(1) = observedCloud->points[i].y + SRORhat.at<double>(1);
		//	SPOR_obs.at<double>(2) = observedCloud->points[i].z + SRORhat.at<double>(2);
			
		//	SPOO_obs = quat2dcm(QROhat).t()*SPOR_obs;
		
			SPRR_sub = quat2dcm(QROhat)*SPRO_sub;
			
    		transformedSubCloud->points[i].x = SPRR_sub.at<double>(0);
			transformedSubCloud->points[i].y = SPRR_sub.at<double>(1);
			transformedSubCloud->points[i].z = SPRR_sub.at<double>(2);
		}
		// Output to user

		// Perform icp on transformedSubCloud -> This produces SReR and QReR (transform from
		// observedCloud to transformedSubCloud).
		icp.setInputSource(observedCloud);
	 	icp.setInputTarget(transformedSubCloud);
	 	
	 	// Perform alignment
	 	icp.align(alignedCloud);
	 	
	 	// Output fitness score and TROhat
	 	ROS_INFO_STREAM("TROhat: " <<  quat2dcm(QROhat));
	 	ROS_INFO_STREAM("SOROhat: " <<  SOROhat);
	 	ROS_INFO_STREAM("Fitness score: " << icp.getFitnessScore());

		// Obtain final transformation from ICP
		// This is error in SORO and TRO
		// If fitness score is < 0.1, no need to do ICP anymore.
		if (icp.getFitnessScore() < 0.01){
			publishLoc = true;
			// Align to alignedCloud
			// Save files
	 		pcl::io::savePCDFileASCII (map_location + "obsCloud.pcd", *observedCloud);
			pcl::io::savePCDFileASCII (map_location + "transformedSub.pcd", *transformedSubCloud);
			pcl::io::savePCDFileASCII (map_location + "aligned.pcd", alignedCloud);
		
		
			std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  			icp.getFitnessScore() << std::endl;
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++) {
					TReR.at<double>(i,j) = icp.getFinalTransformation()(i,j);
				}
				SReRR.at<double>(i) = icp.getFinalTransformation()(i,3);
			}
			errTRO = TReR.t(); //TReR.t();		// QROhat is technically QReOhat
			errSROO = -SReRR;	//(TReR.t()*quat2dcm(QROhat)).t()*SReRR;		// Likewise, SRORhat is SReORehat
			cout << "errTRO: " << errTRO << endl;
			cout << "errSROO: " << errSROO << endl;
			
			// Convert errTOR and errSORO to gmerrOR for broadcast
			errQRO = dcm2quat(errTRO);
		
			gmerrRO.pose.position.x = errSROO.at<double>(0);
			gmerrRO.pose.position.y = errSROO.at<double>(1);
			gmerrRO.pose.position.z = errSROO.at<double>(2);
			gmerrRO.pose.orientation.x = errQRO.at<double>(0);
			gmerrRO.pose.orientation.y = errQRO.at<double>(1);
			gmerrRO.pose.orientation.z = errQRO.at<double>(2);
			gmerrRO.pose.orientation.w = errQRO.at<double>(3);
			gmerrRO.header.stamp = ros::Time::now();
			gmerrRO.header.seq++;
			gmerrRO.header.frame_id = "O-frame";
			
		} else {
			// NOTE:	If observed cloud is used, error terms can be applied to
			//			TORhat/SOROhat due to observedCloud being SPRRhat_obs points.
			//			Error terms will actually be TRO and SORO. Will not need to be applied to QROhat/SRORhat.
			/*// Perform icp on observedCloud
			icp.setInputSource(observedCloud);
		 	icp.setInputTarget(submapCloud);
		 	icp.align(alignedCloud);
		 	std::cout << "Failed with transformedObs. Trying ObservedCloud." << std::endl;
		 	ROS_INFO_STREAM("Fitness score: " << icp.getFitnessScore());
		 	if (icp.getFitnessScore() < 0.01){
				doICP = false;
				publishLoc = true;
				// Align to alignedCloud
				pcl::io::savePCDFileASCII (map_location + "aligned.pcd", alignedCloud);
		
		
				std::cout << "(observedCloud) has converged:" << icp.hasConverged() << " score: " <<
	  			icp.getFitnessScore() << std::endl;
				for (int i = 0; i < 3; i++){
					for (int j = 0; j < 3; j++) {
						errTRO.at<double>(i,j) = icp.getFinalTransformation()(i,j);
					}
					errSROO.at<double>(i) = icp.getFinalTransformation()(i,3);
				}
				cout << "errTRO: " << errTRO << endl;
				cout << "errSROO: " << errSROO << endl;
			}*/
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
	ros::Subscriber submapCloudSub;	// Submap cloud subscriber
	ros::Subscriber cloudAlignEstSub; 	// Subscriber to SRORhat/QROhat
	ros::Publisher localisationUpdate;	// Publisher to send update from localisation node
	
	// Initialisation
	localisationUpdate = nh.advertise<geometry_msgs::PoseStamped>("repeat_node/locPose", 1);
	cloudSub = nh.subscribe("rtabmap/cloud_map", 100, cloudCallBack);	// from RTABMAP
	submapCloudSub = nh.subscribe("repeat_node/submapCloud",1, submapCloudCallBack);	// From repeat pass
	cloudAlignEstSub = nh.subscribe("repeat_node/cloudAlignment",1, cloudAlignmentCallBack);	// SROR, QRO
	// Obtain map location
	nh.getParam("/localisation_node/map_location", map_location);
	
	// main loop
	ros::Rate rate(5);
	while (nh.ok()){
		ros::spinOnce();
		performICP();
		// Broadcast error terms
		// TODO: Will this publish old data? Perhaps if statement could help here.
		if (publishLoc){
			publishLoc = false;
			localisationUpdate.publish(gmerrRO);
		}
		rate.sleep();
	}
	
	
}
