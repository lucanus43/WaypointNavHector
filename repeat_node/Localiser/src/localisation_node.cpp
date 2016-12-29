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

	geometry_msgs::PoseStamped gmRLhat;
	geometry_msgs::PoseStamped gmReRhat;
	bool submapCloudAvailable = false;
	bool observedCloudAvailable = false;
	bool poseAvailable = false;
	bool submapPoseAvailable = false;
	std::string map_location;
	bool publishLoc = false;
	string submapPoseHeaderID;
	string submapCloudHeaderID;
	
// State variables
	Mat SOLLhat = Mat::zeros(3,1,CV_64F);
	Mat QOLhat = Mat::zeros(4,1,CV_64F);
	Mat SRORhat = Mat::zeros(3,1,CV_64F);
	Mat SOROhat = Mat::zeros(3,1,CV_64F);
	Mat QROhat = Mat::zeros(4,1,CV_64F);
		
// ICP variables
	Mat errQRO = Mat::zeros(4,1,CV_64F);
	Mat TReR = Mat::zeros(3,3,CV_64F);
	Mat QReR = Mat::zeros(4,1, CV_64F);
	Mat SReRR = Mat::zeros(3,1,CV_64F);
	Mat errTRO = Mat::zeros(3,3,CV_64F);
	Mat errTOR = Mat::zeros(3,3,CV_64F);
	Mat errSROO = Mat::zeros(3,1,CV_64F);
	Mat TRLhat_icp = Mat::zeros(3,3,CV_64F);
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
	submapCloudHeaderID = input->header.frame_id;
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

// -------------------------------------------------------------
/*
submapPoseCallBack - Callback function for submap pose subscriber

Author: JDev 161220
	
*/
// -------------------------------------------------------------
void submapPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& gmOL){
	// Local variables
	// ..
	
	// Process
	SOLLhat.at<double>(0) = gmOL->pose.position.x;
	SOLLhat.at<double>(1) = gmOL->pose.position.y;
	SOLLhat.at<double>(2) = gmOL->pose.position.z;
	QOLhat.at<double>(0) = gmOL->pose.orientation.x;
	QOLhat.at<double>(1) = gmOL->pose.orientation.y;
	QOLhat.at<double>(2) = gmOL->pose.orientation.z;
	QOLhat.at<double>(3) = gmOL->pose.orientation.w;
	submapPoseHeaderID = gmOL->header.frame_id;
	submapPoseAvailable = true;

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
	Mat TOLhat = Mat::zeros(3,3,CV_64F);
	//Mat SPOR_obs = Mat::zeros(3,1,CV_64F);
	//Mat SPOO_obs = Mat::zeros(3,1,CV_64F);
	
	// Process
	// Set transformedSubCloud to have the same data as submapCloud
	*transformedSubCloud = *submapCloud;
	
	cout << "submapCloudHeaderID: " << submapCloudHeaderID << endl;
	cout << "submapPoseHeaderID: " << submapPoseHeaderID << endl;
	cout << "SOLLhat [loc]: " << SOLLhat << endl;
	
	if ((submapCloudHeaderID.compare(submapPoseHeaderID) == 0) && observedCloudAvailable && poseAvailable){
		// Set poseAvailable, submapCloudAvailable and cloudAvailable to false
		poseAvailable = false;
		observedCloudAvailable = false;
		submapCloudAvailable = false;
		submapPoseAvailable = false;
	
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
	 	pcl::io::savePCDFileASCII (map_location + "obsCloud.pcd", *observedCloud);
		pcl::io::savePCDFileASCII (map_location + "transformedSub.pcd", *transformedSubCloud);
		
		// Perform icp on transformedSubCloud -> This produces SReR and QReR (transform from
		// observedCloud to transformedSubCloud).
		icp.setInputSource(observedCloud);
	 	icp.setInputTarget(transformedSubCloud);
	 	
	 	
	 	// Perform alignment
	 	icp.align(alignedCloud);
	 	
	 	// Output fitness score and TROhat
	 	ROS_INFO_STREAM("TROhat [loc]: " <<  quat2dcm(QROhat));
	 	ROS_INFO_STREAM("SOROhat [loc]: " <<  SOROhat);
	 	ROS_INFO_STREAM("SOLLhat [loc]: " << SOLLhat);
	 	ROS_INFO_STREAM("Fitness score [loc]: " << icp.getFitnessScore());

		// Obtain final transformation from ICP
		// This is error in SORO and TRO
		// If fitness score is < 0.1, no need to do ICP anymore.
		if (icp.getFitnessScore() < 0.05){
			publishLoc = true;
			// Align to alignedCloud
			// Save files

			pcl::io::savePCDFileASCII (map_location + "aligned.pcd", alignedCloud);
		
		
			std::cout << "[loc] has converged:" << icp.hasConverged() << " score: " <<
  			icp.getFitnessScore() << std::endl;
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++) {
					TReR.at<double>(i,j) = icp.getFinalTransformation()(i,j);
				}
				SReRR.at<double>(i) = icp.getFinalTransformation()(i,3);
			}
			
			// Set QReR
			QReR = dcm2quat(TReR);
			
			errTRO = TReR.t(); //TReR.t();		// QROhat is technically QReOhat
			errSROO = -SReRR;	//(TReR.t()*quat2dcm(QROhat)).t()*SReRR;		// Likewise, SRORhat is SReORehat
			ROS_INFO_STREAM("errTRO [loc]: " << errTRO);
			ROS_INFO_STREAM("errSROO [loc]: " << errSROO);
			
			// Convert errTOR and errSORO to gmerrOR for broadcast
			errQRO = dcm2quat(errTRO);
			cout << "SOLLhat [loc]: " << SOLLhat << endl;
			
			// Convert QOLhat to TOLhat
			TOLhat = quat2dcm(QOLhat);
			
			// Convert errQRO to errTOR
			errTOR = quat2dcm(errQRO).t(); 		// errTOR = TReR;
			TORhat_icp = quat2dcm(QROhat).t()*errTOR;		// TORhat = TORe*TReR		// QROhat <- state update
			cout << "QORhat_icp: " << dcm2quat(TORhat_icp) << endl;
		
			// errSROO = SReRR; SORRhat = SOReR+SReRR
			SORRhat_icp = - errTOR*SRORhat + errSROO;		//-TORhat_icp.t()*errSROO - SRORhat;	// SORRhat <- state update
			cout << "SORRhat_icp: " << SORRhat_icp << endl;
		
			// Calculate QRLhat_icp, SRLLhat_icp
			QRLhat_icp = dcm2quat(TORhat_icp.t()*TOLhat);			// TOLhat <- submap
			cout << "QRLhat_icp: " << QRLhat_icp << endl;
		
			SORLhat_icp = quat2dcm(QRLhat_icp).t()*SORRhat_icp;
			cout << "SORLhat_icp: " << SORLhat_icp << endl;
		
			SRLLhat_icp = -SORLhat_icp + SOLLhat; 				// SOLLhat <- submap
			cout << "SRLLhat_icp: " << SRLLhat_icp << endl;
			
			
			// Set SRLLhat_icp and QRLhat_icp for broadcast
			gmRLhat.pose.position.x = SRLLhat_icp.at<double>(0);
			gmRLhat.pose.position.y = SRLLhat_icp.at<double>(1);
			gmRLhat.pose.position.z = SRLLhat_icp.at<double>(2);
			gmRLhat.pose.orientation.x = QRLhat_icp.at<double>(0);
			gmRLhat.pose.orientation.y = QRLhat_icp.at<double>(1);
			gmRLhat.pose.orientation.z = QRLhat_icp.at<double>(2);
			gmRLhat.pose.orientation.w = QRLhat_icp.at<double>(3);
			gmRLhat.header.stamp = ros::Time::now();
			gmRLhat.header.seq++;
			gmRLhat.header.frame_id = submapCloudHeaderID;
			
			
			// Set SReR and QReR for broadcast
			gmReRhat.pose.position.x = SReRR.at<double>(0);
			gmReRhat.pose.position.y = SReRR.at<double>(1);
			gmReRhat.pose.position.z = SReRR.at<double>(2);
			gmReRhat.pose.orientation.x = QReR.at<double>(0);
			gmReRhat.pose.orientation.y = QReR.at<double>(1);
			gmReRhat.pose.orientation.z = QReR.at<double>(2);
			gmReRhat.pose.orientation.w = QReR.at<double>(3);
			gmReRhat.header.stamp = ros::Time::now();
			gmReRhat.header.seq++;
			gmReRhat.header.frame_id = submapCloudHeaderID;
			
		} else {
			// NOTE:	If observed cloud is used, error terms can be applied to
			//			TORhat/SOROhat due to observedCloud being SPRRhat_obs points.
			//			Error terms will actually be TRO and SORO. Will not need to be applied to QROhat/SRORhat.
			// Perform icp on observedCloud
			/*icp.setInputSource(observedCloud);
		 	icp.setInputTarget(submapCloud);
		 	icp.align(alignedCloud);
		 	std::cout << "Failed with transformedObs. Trying ObservedCloud." << std::endl;
		 	ROS_INFO_STREAM("Fitness score: " << icp.getFitnessScore());
		 	if (icp.getFitnessScore() < 0.01){
				publishLoc = true;
				// Align to alignedCloud
				pcl::io::savePCDFileASCII (map_location + "aligned.pcd", alignedCloud);
		
		
				std::cout << "(observedCloud) has converged:" << icp.hasConverged() << " score: " <<
	  			icp.getFitnessScore() << std::endl;
				for (int i = 0; i < 3; i++){
					for (int j = 0; j < 3; j++) {
						TRLhat_icp.at<double>(i,j) = icp.getFinalTransformation()(i,j);
					}
					SRLLhat_icp.at<double>(i) = icp.getFinalTransformation()(i,3);
				}
				cout << "QRLhat_icp: " << dcm2quat(TRLhat_icp) << endl;
				cout << "SRLLhat_icp: " << SRLLhat_icp << endl;
			
					// Set SRLLhat_icp and QRLhat_icp for broadcast
			gmRLhat.pose.position.x = SRLLhat_icp.at<double>(0);
			gmRLhat.pose.position.y = SRLLhat_icp.at<double>(1);
			gmRLhat.pose.position.z = SRLLhat_icp.at<double>(2);
			gmRLhat.pose.orientation.x = QRLhat_icp.at<double>(0);
			gmRLhat.pose.orientation.y = QRLhat_icp.at<double>(1);
			gmRLhat.pose.orientation.z = QRLhat_icp.at<double>(2);
			gmRLhat.pose.orientation.w = QRLhat_icp.at<double>(3);
			gmRLhat.header.stamp = ros::Time::now();
			gmRLhat.header.seq++;
			gmRLhat.header.frame_id = submapCloudHeaderID;
			
			// Set SReR and QReR for broadcast
			gmReRhat.pose.position.x = SReRR.at<double>(0);
			gmReRhat.pose.position.y = SReRR.at<double>(1);
			gmReRhat.pose.position.z = SReRR.at<double>(2);
			gmReRhat.pose.orientation.x = QReR.at<double>(0);
			gmReRhat.pose.orientation.y = QReR.at<double>(1);
			gmReRhat.pose.orientation.z = QReR.at<double>(2);
			gmReRhat.pose.orientation.w = QReR.at<double>(3);
			gmReRhat.header.stamp = ros::Time::now();
			gmReRhat.header.seq++;
			gmReRhat.header.frame_id = submapCloudHeaderID;
			
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
	ros::Subscriber submapPoseSub;		// Subscriber to SOLLhat/QOLhat
	ros::Publisher localisationErrorPub;	// Localisation error publisher
	ros::Publisher localisationUpdate;	// Publisher to send update from localisation node
	
	// Initialisation
	localisationUpdate = nh.advertise<geometry_msgs::PoseStamped>("repeat_node/locPose", 1);
	localisationErrorPub = nh.advertise<geometry_msgs::PoseStamped>("repeat_node/locErr", 1);
	cloudSub = nh.subscribe("rtabmap/cloud_map", 1, cloudCallBack);	// from RTABMAP
	submapCloudSub = nh.subscribe("repeat_node/submapCloud",1, submapCloudCallBack);	// From repeat pass
	cloudAlignEstSub = nh.subscribe("repeat_node/cloudAlignment",1, cloudAlignmentCallBack);	// SROR, QRO
	submapPoseSub = nh.subscribe("repeat_node/submapPose",1, submapPoseCallBack);
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
			localisationUpdate.publish(gmRLhat);
			localisationErrorPub.publish(gmReRhat);
		}
		rate.sleep();
	}
	
	
}
