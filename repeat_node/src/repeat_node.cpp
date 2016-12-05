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
	Mat SCLL_cmd;
	Mat SBLL_cmd = Mat::zeros(3,1,CV_64F);
	Mat TCL_cmd;
	Mat QCL_cmd;
	double wp_radius = 0.1;
	vector<geometry_msgs::Pose> waypointsCL;
	int wp_counter = 1;
	bool next_map = true;
	bool doICP = true;
	Mat QBL_cmd = Mat::zeros(4,1,CV_64F);
	geometry_msgs::PoseStamped gmBL_cmd;
	geometry_msgs::PoseWithCovarianceStamped gmBLhat;
	pcl::PointCloud<pcl::PointXYZ>::Ptr submapCloud (new pcl::PointCloud<pcl::PointXYZ>);
	
// Truth variables
	Mat QCB = Mat::zeros(4,1,CV_64F);
	Mat TCB = Mat::zeros(3,3,CV_64F);
	Mat SCBB = Mat::zeros(3,1,CV_64F);
	Mat SBLL = Mat::zeros(3,1,CV_64F);
	Mat QBL = Mat::zeros(4,1,CV_64F);
	
// Teach node variables
	Mat TOLhat = Mat::zeros(3,3,CV_64F);
	Mat SOLLhat = Mat::zeros(3,1,CV_64F);
	Mat QOLhat = Mat::zeros(4,1,CV_64F);

	
// VO variables
	rtabmap_ros::ResetPose::Request odomResetReq;
	rtabmap_ros::ResetPose::Response odomResetResp;
	bool clearMap = true;
	Mat SCRChat_vo = Mat::zeros(3,1,CV_64F);
	Mat QCRhat_vo = Mat::zeros(4,1,CV_64F);
	Mat TRLhat_vo = Mat::zeros(3,3,CV_64F);
	Mat SRLLhat_vo = Mat::zeros(3,1,CV_64F);
	Mat TCLhat_vo = Mat::zeros(3,3,CV_64F);
	Mat SCLLhat_vo = Mat::zeros(3,1,CV_64F);
	Mat QBLhat_vo = Mat::zeros(4,1,CV_64F);
	Mat SBLLhat_vo = Mat::zeros(3,1,CV_64F);
	Mat QCLhat_vo = Mat::zeros(4,1,CV_64F);
	Mat QRLhat_vo = Mat::zeros(4,1,CV_64F);
	int VOLossCounter = 0;
	
// INS variables
	Mat QRLhat_ins = Mat::zeros(4,1,CV_64F);
	Mat SRLLhat_ins = Mat::zeros(3,1,CV_64F);
	Mat TCLhat_ins = Mat::zeros(3,3,CV_64F);
	Mat SCLLhat_ins = Mat::zeros(3,1,CV_64F);
	Mat QBLhat_ins = Mat::zeros(4,1,CV_64F);
	Mat SBLLhat_ins = Mat::zeros(3,1,CV_64F);
	Mat SCRLhat_ins = Mat::zeros(3,1,CV_64F);
	Mat QCLhat_ins = Mat::zeros(4,1,CV_64F);
	Mat QCRhat_ins = Mat::zeros(4,1,CV_64F);
	
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
	
// State variables
	Mat SCRChat = Mat::zeros(3,1,CV_64F);
	Mat SCRLhat = Mat::zeros(3,1,CV_64F);
	Mat QCRhat = Mat::zeros(4,1,CV_64F);
	Mat SRLLhat = Mat::zeros(3,1,CV_64F);
	Mat SCLLhat = Mat::zeros(3,1,CV_64F);
	Mat QBLhat = Mat::zeros(4,1,CV_64F);
	Mat SBLLhat = Mat::zeros(3,1,CV_64F);
	Mat QRLhat = Mat::zeros(4,1,CV_64F);
	Mat QCLhat = Mat::zeros(4,1,CV_64F);
	Mat SRORhat = Mat::zeros(3,1,CV_64F);
	Mat QROhat = Mat::zeros(4,1,CV_64F);
	
	
typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseActionClient;

// Function prototypes
void updateState(Mat inSBLLhat, Mat inQBLhat, Mat inSRLLhat, Mat inQRLhat);
void resetMap();
void resetVO();
	
// ---------------------------- CALLBACK FUNCTIONS ---------------------------//	
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr observedCloud(new pcl::PointCloud<pcl::PointXYZ>); // temp_cloud is in PointXYZ form
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedObsCloud(new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PointCloud<pcl::PointXYZ> alignedCloud;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	Mat errTOR = Mat::zeros(3,3,CV_64F);
	Mat errSORO = Mat::zeros(3,1,CV_64F);
	Mat SPOR_obs = Mat::zeros(3,1,CV_64F);
	Mat SPOO_obs = Mat::zeros(3,1,CV_64F);
	
	if (doICP){
		
		pcl_conversions::toPCL(*input,pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2,*observedCloud);
		observedCloud->is_dense = false;
		// Set transformedObsCloud to have the same data as observedCloud
		*transformedObsCloud = *observedCloud;
		
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
			// Align to alignedCloud
			doICP = false;
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
		
			// Calculate SORRhat_icp, TROhat_icp
			TORhat_icp = errTOR*quat2dcm(QROhat).t();				// QROhat <- state update
			cout << "QORhat_icp: " << dcm2quat(TORhat_icp) << endl;
			cout << "QORhat: " << dcm2quat(quat2dcm(QROhat).t()) << endl;
			
			SORRhat_icp = TORhat_icp.t()*errSORO - SRORhat;			// SORRhat <- state update
			cout << "SORRhat_icp: " << SORRhat_icp << endl;
			cout << "SORRhat: " << -SRORhat << endl;
			
			// Calculate QRLhat_icp, SRLLhat_icp
			QRLhat_icp = dcm2quat(TORhat_icp.t()*TOLhat);			// TOLhat <- submap
			cout << "QRLhat_icp: " << QRLhat_icp << endl;
			cout << "QRLhat: " << QRLhat << endl;
			
			SORLhat_icp = quat2dcm(QRLhat_icp).t()*SORRhat_icp;
			cout << "SORLhat_icp: " << SORLhat_icp << endl;
			cout << "SORLhat" << -quat2dcm(QRLhat).t()*SRORhat << endl;
			
			SRLLhat_icp = -SORLhat_icp + SOLLhat; 				// SOLLhat <- submap
			cout << "SRLLhat_icp: " << SRLLhat_icp << endl;
			cout << "SRLLhat: " << SRLLhat << endl;
			// Calculate SCLLhat_icp, TCLhat_icp
			TCLhat_icp = quat2dcm(QCRhat)*quat2dcm(QRLhat_icp);			// QCRhat <- state update
			SCLLhat_icp = TCLhat_icp.t()*SCRChat + SRLLhat_icp;	// SCRChat <- state update	
			// Calculate SBLLhat_icp, TBLhat_icp
			SBLLhat_icp = -TCLhat_icp.t()*TCB*SCBB+SCLLhat_icp;	// TCB/SCBB <- Known
			QBLhat_icp = dcm2quat(TCB.t()*TCLhat_icp);
			ROS_INFO("SBLLhat_icp: [%f,%f,%f]", SBLLhat_icp.at<double>(0), SBLLhat_icp.at<double>(1), SBLLhat_icp.at<double>(2));
			ROS_INFO("SBLL: [%f,%f,%f]", SBLL.at<double>(0), SBLL.at<double>(1), SBLL.at<double>(2));
			// Perform state update
			//updateState(SBLLhat_icp, QBLhat_icp, SRLLhat_icp, QRLhat_icp);
			resetMap();
		} else {
			resetMap();
		}
		
	}
}



// -------------------------------------------------------------
/*
voCallBack- 	Called when odometry is received.
				R: Repeat pass odometry frame
		

Author: JDev 161125
	
*/
// -------------------------------------------------------------

void voCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg){
	// Local variables
	// ..
	
	// Process
	// Obtain SCRC
	SCRChat_vo.at<double>(0) = (odomMsg->pose.pose.position.x);
	SCRChat_vo.at<double>(1) = (odomMsg->pose.pose.position.y);
	SCRChat_vo.at<double>(2) = (odomMsg->pose.pose.position.z);
	
	// Get QCR from quaternion in odomMsg
	QCRhat_vo.at<double>(0) = (odomMsg->pose.pose.orientation.x); 
	QCRhat_vo.at<double>(1) = (odomMsg->pose.pose.orientation.y); 
	QCRhat_vo.at<double>(2) = (odomMsg->pose.pose.orientation.z); 
	QCRhat_vo.at<double>(3) = (odomMsg->pose.pose.orientation.w);
	
	// If SCRC is zero (VO lost), increment VO loss counter. SBLLhat will not be set to zero.
	// Else, perform VO as per normal.
	if ( fabs(norm(SCRChat_vo)) == 0){
		ROS_INFO_STREAM("VO lost. Incrementing counter.");
		VOLossCounter++;
	} else {
		// Use VO alone to update state
		TCLhat_vo = quat2dcm(QCRhat_vo)*quat2dcm(QRLhat);	// TCB*quat2dcm(QBLhat);					// QBLhat <- state update
		TRLhat_vo = quat2dcm(QCRhat_vo).t()*TCLhat_vo;				
		SCLLhat_vo = TCLhat_vo.t()*SCRChat_vo + SRLLhat; //TCLhat_vo.t()*TCB*SCBB + SBLLhat;		// SRLLhat <- state update
		// TODO: Check if SRLLhat_vo stays constant:
		SRLLhat_vo = -TCLhat_vo.t()*SCRChat_vo + SCLLhat_vo;	// SRLL = TLC*SRCC + SCLL	
		
		// Calculate SBLLhat_vo, QBLhat_vo, SRLLhat_vo and QRLhat_vo
		QBLhat_vo =  dcm2quat(TCB.t()*TCLhat_vo);				
		SBLLhat_vo = -quat2dcm(QBLhat_vo).t()*SCBB + SCLLhat_vo;	
		//ROS_INFO("SCRChat_vo: [%f,%f,%f]", SCRChat_vo.at<double>(0), SCRChat_vo.at<double>(1), SCRChat_vo.at<double>(2));
		ROS_INFO("SBLLhat_vo: [%f,%f,%f]", SBLLhat_vo.at<double>(0), SBLLhat_vo.at<double>(1), SBLLhat_vo.at<double>(2));
		// Send to state update
		updateState(SBLLhat_vo, QBLhat_vo, SRLLhat_vo, dcm2quat(TRLhat_vo));
	}
	
}


// -------------------------------------------------------------
/*
insCallBack- 	Called when SBLLhat/QBLhat is received
					

Author: JDev 161201
	
*/
// -------------------------------------------------------------
void insCallBack( const geometry_msgs::PoseStamped::ConstPtr& extPose ){
	// Local variables
	// ..
	
	// Update ins variables
	SBLLhat_ins.at<double>(0) = extPose->pose.position.x;
	SBLLhat_ins.at<double>(1) = extPose->pose.position.y;
	SBLLhat_ins.at<double>(2) = extPose->pose.position.z;
	QBLhat_ins.at<double>(0) = extPose->pose.orientation.x;
	QBLhat_ins.at<double>(1) = extPose->pose.orientation.y;
	QBLhat_ins.at<double>(2) = extPose->pose.orientation.z;
	QBLhat_ins.at<double>(3) = extPose->pose.orientation.w;
	
	// Calculate SRLLhat_ins and TRLhat_ins
	SCLLhat_ins = quat2dcm(QBLhat_ins).t()*SCBB+SBLLhat_ins;
	SCRLhat_ins = SCLLhat_ins - SRLLhat;				// SRLLhat <- State update	
	// TODO: Check if SRLLhat_ins, QCRhat_ins stays constant
	SRLLhat_ins = -SCRLhat_ins + SCLLhat_ins;						// SCRLhat <- State update
	QCLhat_ins = dcm2quat(TCB*quat2dcm(QBLhat_ins));
	QCRhat_ins = dcm2quat(quat2dcm(QCLhat_ins)*quat2dcm(QRLhat).t());		// QRLhat <- State update
	// TRL = TRC*TCB*TBL
	QRLhat_ins = dcm2quat(quat2dcm(QCRhat_ins).t()*TCB*quat2dcm(QBLhat_ins));	
	
	//ROS_INFO("SBLLhat_ins: [%f,%f,%f]", SBLLhat_ins.at<double>(0), SBLLhat_ins.at<double>(1), SBLLhat_ins.at<double>(2));
	// Send to state update
	updateState(SBLLhat_ins, QBLhat_ins, SRLLhat_ins, QRLhat_ins);
}


// -------------------------------------------------------------
/*
truePositionCallBack- 	Called when SBLL/TBL is received (listening for TBL - truth)
					

Author: JDev 161201
	
*/
// -------------------------------------------------------------
void truePositionCallBack( const geometry_msgs::PoseStamped::ConstPtr& gmBL ) {
	// Local Variables
	
	// Truth 
	SBLL.at<double>(0) = gmBL->pose.position.x;
	SBLL.at<double>(1) = gmBL->pose.position.y;
	SBLL.at<double>(2) = gmBL->pose.position.z;
	QBL.at<double>(0) = gmBL->pose.orientation.x;
	QBL.at<double>(1) = gmBL->pose.orientation.y;
	QBL.at<double>(2) = gmBL->pose.orientation.z;
	QBL.at<double>(3) = gmBL->pose.orientation.w;
	
	
}
	
	
	
// ---------------------------- PROCESS FUNCTIONS ---------------------------//	

// -------------------------------------------------------------
/*
generateWaypoints -	Generates waypoints from the given poses.txt file
					Called every time a submap is loaded.
					
					NOTE: Uses globals SCLL_cmd and QCL_cmd. This should not affect
					the variable used in the rest of the script since this is only
					called at the beginning and the values will be changed in the first
					positionCallBack.
					TODO: Check if this is true.

Author: JDev 161125
	
*/
// -------------------------------------------------------------
void generateWaypoints(string poseFileName){
	// Local variables
	fstream poseFile;
	vector<Mat> vecSCOL;
	vector<Mat> vecQCL;
	geometry_msgs::Pose temp_wp;

	
	// Read in waypoints from poseFile along with SOLLhat
	poseFile.open(poseFileName.c_str());
	if(poseFile.fail()){
		ROS_ERROR_STREAM("Error: File stream " << poseFileName << " failed to open (check spelling)");
		ros::shutdown(); 
	}
	
	// Generate waypoints as SCLL_cmd and QCL_cmd using SCOLhat and SOLLhat and QCL from pose file
		// SCLL_cmd = SCOLhat + SOLLhat
		// QCL_cmd = posefile.QCL
	extractPosesFromFile(poseFile, SOLLhat, QOLhat, vecSCOL, vecQCL);
	
	// Subsample pose file
	// .. TODO
	
	// Set TOLhat
	TOLhat = quat2dcm(QOLhat);
	
	// Make first waypoint a takeoff waypoint
	temp_wp.position.x = 0;
	temp_wp.position.y = 0;
	temp_wp.position.z = 5;
	temp_wp.orientation.x = vecQCL[0].at<double>(0);
	temp_wp.orientation.y = vecQCL[0].at<double>(1);
	temp_wp.orientation.z = vecQCL[0].at<double>(2);
	temp_wp.orientation.w = vecQCL[0].at<double>(3);
	waypointsCL.push_back(temp_wp);
	
	// Create waypoints
	for (int i = 0; i < vecSCOL.size(); i++){
		// Commanded displacement vector rel. Local Level frame
		SCLL_cmd = vecSCOL[i] + SOLLhat;
		QCL_cmd = vecQCL[i];
		// Geometry message
		temp_wp.position.x = SCLL_cmd.at<double>(0);
		temp_wp.position.y = SCLL_cmd.at<double>(1);
		temp_wp.position.z = SCLL_cmd.at<double>(2);
		temp_wp.orientation.x = QCL_cmd.at<double>(0);
		temp_wp.orientation.y = QCL_cmd.at<double>(1);
		temp_wp.orientation.z = QCL_cmd.at<double>(2);
		temp_wp.orientation.w = QCL_cmd.at<double>(3);
		// Waypoints (pose of C wrt. L)
		waypointsCL.push_back(temp_wp);
	}
}

// -------------------------------------------------------------
/*
loadSubmapPCD -	Loads a Pointcloud into memory
				Adapted from: http://pointclouds.org/documentation/tutorials/reading_pcd.php

Author: JDev 161125
	
*/
// -------------------------------------------------------------
void loadSubmapPCD(string PCDFileName){
	// Local variables
	// ..
	
	// Load submap into a pointxyz variable
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (PCDFileName.c_str(), *submapCloud) == -1) //* load the file
	  {
		ROS_ERROR_STREAM("Couldn't read file " << PCDFileName.c_str() << ". Exiting.");
		ros::shutdown(); 
	  }
	// Set submapCloud params
	submapCloud->is_dense = false;
}

// -------------------------------------------------------------
/*
initVars -	Initialises all variables

Author: JDev 161201
	
*/
// -------------------------------------------------------------
bool initVars(){
	// Local Variables
	// ..
	
	// Set fixed transforms TCB/SCBB
	QCB.at<double>(1) = 0.707; QCB.at<double>(3) = 0.707;
	TCB = quat2dcm(QCB);
	// Set SCBB
	SCBB.at<double>(0) = 0.1;
	SCBB.at<double>(2) = -0.03;
	
	// Initialise state variables
	// SBLLhat, QBLhat
	SBLLhat = SBLL.clone(); QBLhat = QBL.clone();
	SCLLhat = quat2dcm(QBL).t()*SCBB+SBLL;
	QCLhat = dcm2quat(TCB*quat2dcm(QBL));
	SRLLhat = quat2dcm(QBL).t()*SCBB+SBLL;
	QRLhat = dcm2quat(TCB*quat2dcm(QBL));
	
	// Initialise INS variables
	// SCLLhat_ins, QCLhat_ins, SBLLhat_ins, QBLhat_ins
	SBLLhat_ins = SBLL.clone();
	QBLhat_ins = QBL.clone();
	SCLLhat_ins = quat2dcm(QBL).t()*SCBB+SBLL;
	QCLhat_ins = dcm2quat(TCB*quat2dcm(QBL));
	
	
	// Initialise VO variables
	// SCLLhat_vo, QCLhat_vo, SBLLhat_vo, QBLhat_vo, SRLLhat_vo, QRLhat_vo
	SBLLhat_vo = SBLL.clone();
	QBLhat_vo = QBL.clone();
	SCLLhat_vo = quat2dcm(QBL).t()*SCBB+SBLL;
	QCLhat_vo = dcm2quat(TCB*quat2dcm(QBL));
	SRLLhat_vo = quat2dcm(QBL).t()*SCBB+SBLL;	// SRLL = initial SCLL
	QRLhat_vo = dcm2quat(TCB*quat2dcm(QBL));	// QRL = initial QCL
	
	ROS_INFO_STREAM("TBL init: " << quat2dcm(QBL));
	ROS_INFO_STREAM("TRLhat_vo init: " << quat2dcm(QRLhat_vo));
	ROS_INFO_STREAM("TCB init: " << TCB);
	
	// Initialise ICP variables
	SBLLhat_icp = SBLL.clone();
	QBLhat_icp = QBL.clone();
	SCLLhat_icp = quat2dcm(QBL).t()*SCBB+SBLL;
	QCLhat_icp = dcm2quat(TCB*quat2dcm(QBL));
	SRLLhat_icp = quat2dcm(QBL).t()*SCBB+SBLL;
	QRLhat_icp = dcm2quat(TCB*quat2dcm(QBL));	
	
	// Since SRLLhat/QRLhat has been set, reset map to reflect this.
	resetVO();
	resetMap();

		
	// Set first waypoint
	SCLL_cmd.at<double>(0) = waypointsCL.at(wp_counter).position.x;
	SCLL_cmd.at<double>(1) = waypointsCL.at(wp_counter).position.y;
	SCLL_cmd.at<double>(2) = waypointsCL.at(wp_counter).position.z;
	QCL_cmd.at<double>(0) = waypointsCL.at(wp_counter).orientation.x;
	QCL_cmd.at<double>(1) = waypointsCL.at(wp_counter).orientation.y;
	QCL_cmd.at<double>(2) = waypointsCL.at(wp_counter).orientation.z;
	QCL_cmd.at<double>(3) = waypointsCL.at(wp_counter).orientation.w;
	// Obtain SBLL_cmd, QBL_cmd
	SBLL_cmd = quat2dcm(QBLhat).t()*SCBB + SCLL_cmd;
	QBL_cmd = dcm2quat(TCB.t()*quat2dcm(QCL_cmd));
	// Export commanded pose as gmBL_cmd
	gmBL_cmd.pose.position.x = SBLL_cmd.at<double>(0);
	gmBL_cmd.pose.position.y = SBLL_cmd.at<double>(1);
	gmBL_cmd.pose.position.z = SBLL_cmd.at<double>(2);
	gmBL_cmd.pose.orientation.x = QBL_cmd.at<double>(0);
	gmBL_cmd.pose.orientation.y = QBL_cmd.at<double>(1);
	gmBL_cmd.pose.orientation.z = QBL_cmd.at<double>(2);
	gmBL_cmd.pose.orientation.w = QBL_cmd.at<double>(3);
	gmBL_cmd.header.stamp = ros::Time::now();
	gmBL_cmd.header.seq++;
	gmBL_cmd.header.frame_id = "world";
	
	// Return true if SBLL is not zero
	if (fabs(norm(SBLL)) > 0){
		// Update state (just in case), use any suffix. All variables are the same.
		updateState(SBLLhat, QBLhat, SRLLhat_vo, QRLhat_vo);	
		ROS_INFO("SRLLhat init: [%f,%f,%f]", SRLLhat_vo.at<double>(0), SRLLhat_vo.at<double>(1), SRLLhat_vo.at<double>(2));
		ROS_INFO("SBLL populated, exiting initialisation.");
		return true;
	} else {
		return false;
	}
	
}

// -------------------------------------------------------------
/*
updateState -	Updates state variables

Author: JDev 161201
	
*/
// -------------------------------------------------------------

void updateState(Mat inSBLLhat, Mat inQBLhat, Mat inSRLLhat, Mat inQRLhat){
	// Local Variables
	// ..
	
	// Average current state with input state
	SBLLhat = 0.5*(SBLLhat + inSBLLhat);
	QBLhat = 0.5*(QBLhat + inQBLhat);
	SRLLhat = 0.5*(SRLLhat + inSRLLhat);
	QRLhat = 0.5*(QRLhat + inQRLhat);
	
	// Update all state variables with new SBLLhat, QBLhat
	SCLLhat = quat2dcm(QBLhat).t()*SCBB + SBLLhat;
	SCRLhat = SCLLhat - SRLLhat;
	// ..
	QCLhat = dcm2quat(TCB*quat2dcm(QBLhat));
	QCRhat = dcm2quat(quat2dcm(QCLhat)*quat2dcm(QRLhat).t());
	//..
	SCRChat = quat2dcm(QCLhat)*SCRLhat;
	// ..
	SRORhat = quat2dcm(QRLhat)*(SRLLhat - SOLLhat);		// SOLLhat <- Known		
	QROhat = dcm2quat(quat2dcm(QRLhat)*TOLhat.t());
	
	//




	
	
	//ROS_INFO_STREAM("TROhat: " << quat2dcm(QROhat));
	//ROS_INFO_STREAM("TRLhat: " << quat2dcm(QRLhat));
	//ROS_INFO("SRLLhat: [%f,%f,%f]", SRLLhat.at<double>(0), SRLLhat.at<double>(1), SRLLhat.at<double>(2));
	ROS_INFO("SBLLhat: [%f,%f,%f]", SBLLhat.at<double>(0), SBLLhat.at<double>(1), SBLLhat.at<double>(2));
	//ROS_INFO("QRLhat: [%f,%f,%f,%f]", QRLhat.at<double>(0), QRLhat.at<double>(1), QRLhat.at<double>(2), QRLhat.at<double>(3));
	// gmBLhat for publishing
	gmBLhat.pose.pose.position.x = SBLLhat.at<double>(0);
	gmBLhat.pose.pose.position.y = SBLLhat.at<double>(1);
	gmBLhat.pose.pose.position.z = SBLLhat.at<double>(2);
	gmBLhat.pose.pose.orientation.x = QBLhat.at<double>(0);
	gmBLhat.pose.pose.orientation.y = QBLhat.at<double>(1);
	gmBLhat.pose.pose.orientation.z = QBLhat.at<double>(2);
	gmBLhat.pose.pose.orientation.w = QBLhat.at<double>(3);
	// Set covariance to arbitrary value such that it is accepted by /poseupdate topic
	// TODO: Work out the actual covariance
	gmBLhat.pose.covariance[0] = 0.5;
	gmBLhat.pose.covariance[7] = 0.5;
	gmBLhat.pose.covariance[14] = 0.5;
	gmBLhat.pose.covariance[21] = 0.5;
	gmBLhat.pose.covariance[28] = 0.5;
	gmBLhat.pose.covariance[35] = 0.5;
	// gmBLhat header
	gmBLhat.header.stamp = ros::Time::now();
	gmBLhat.header.seq++;
	gmBLhat.header.frame_id = "/nav";	
	
}



// -------------------------------------------------------------
/*
nextMap -	Loads the next submap, if there is one.

Author: JDev 161201
	
*/
// -------------------------------------------------------------
void nextMap(){
}


// -------------------------------------------------------------
/*
waypointNav -	Checks if we can proceed to next waypoint.

Author: JDev 161201
	
*/
// -------------------------------------------------------------
void waypointNav(){
	// Local Variables
	// ..
	
	// Process
	// Check to see if waypoints have been reached.
	//ROS_INFO("SCLL_cmd: [%f,%f,%f]", SCLL_cmd.at<double>(0), SCLL_cmd.at<double>(1), SCLL_cmd.at<double>(2));
	//ROS_INFO("SCLLhat: [%f,%f,%f]", SCLLhat.at<double>(0), SCLLhat.at<double>(1), SCLLhat.at<double>(2));
	ROS_INFO("SBLL_cmd: [%f,%f,%f]", SBLL_cmd.at<double>(0), SBLL_cmd.at<double>(1), SBLL_cmd.at<double>(2));
	if( fabs( SCLLhat.at<double>(0) -  SCLL_cmd.at<double>(0) ) < wp_radius && fabs( QCLhat.at<double>(0) -  QCL_cmd.at<double>(0) ) < wp_radius) {
		if( fabs( SCLLhat.at<double>(1) - SCLL_cmd.at<double>(1))  < wp_radius && fabs( QCLhat.at<double>(1) -  QCL_cmd.at<double>(1) ) < wp_radius) {
			if( fabs( SCLLhat.at<double>(2) - SCLL_cmd.at<double>(2) )  < wp_radius && fabs( QCLhat.at<double>(2) -  QCL_cmd.at<double>(2) ) < wp_radius) {
				//If there are more waypoints
				wp_counter++;	//Move to the next waypoint
				if( wp_counter < waypointsCL.size() ) {
					SCLL_cmd.at<double>(0) = waypointsCL.at(wp_counter).position.x;
					SCLL_cmd.at<double>(1) = waypointsCL.at(wp_counter).position.y;
					SCLL_cmd.at<double>(2) = waypointsCL.at(wp_counter).position.z;
					QCL_cmd.at<double>(0) = waypointsCL.at(wp_counter).orientation.x;
					QCL_cmd.at<double>(1) = waypointsCL.at(wp_counter).orientation.y;
					QCL_cmd.at<double>(2) = waypointsCL.at(wp_counter).orientation.z;
					QCL_cmd.at<double>(3) = waypointsCL.at(wp_counter).orientation.w;
					SBLL_cmd = quat2dcm(QBLhat).t()*SCBB + SCLL_cmd;
					QBL_cmd = dcm2quat(TCB.t()*quat2dcm(QCL_cmd));
					// gmBL_cmd
					gmBL_cmd.pose.position.x = SBLL_cmd.at<double>(0);
					gmBL_cmd.pose.position.y = SBLL_cmd.at<double>(1);
					gmBL_cmd.pose.position.z = SBLL_cmd.at<double>(2);
					gmBL_cmd.pose.orientation.x = QBL_cmd.at<double>(0);
					gmBL_cmd.pose.orientation.y = QBL_cmd.at<double>(1);
					gmBL_cmd.pose.orientation.z = QBL_cmd.at<double>(2);
					gmBL_cmd.pose.orientation.w = QBL_cmd.at<double>(3);
					// gmBL_cmd Header
					gmBL_cmd.header.stamp = ros::Time::now();
					gmBL_cmd.header.seq++;
					gmBL_cmd.header.frame_id = "world";
				} else {
					next_map = true;
					ROS_INFO( "Finished the waypoint path!" );
					// DEBUG TODO: get rid of
					ros::shutdown();
				}
			}
		}
	}
}

// -------------------------------------------------------------
/*
resetVO -	Resets visual odometry

Author: JDev 161201
	
*/
// -------------------------------------------------------------
void resetVO(){
	// Local variables
	double QCRhatroll, QCRhatpitch, QCRhatyaw;
	tf::Quaternion tfQCRhat;
	geometry_msgs::Quaternion gmQCRhat;
	
	// Get get camera angles
	gmQCRhat.x = QCRhat.at<double>(0); gmQCRhat.y = QCRhat.at<double>(1); gmQCRhat.z = QCRhat.at<double>(2); gmQCRhat.w = QCRhat.at<double>(3); 
   	tf::quaternionMsgToTF(gmQCRhat, tfQCRhat);
    tf::Matrix3x3(tfQCRhat).getEulerYPR(QCRhatyaw, QCRhatpitch, QCRhatroll);
	
	// Set request for reset
	odomResetReq.x = SCRChat.at<double>(0); odomResetReq.y = SCRChat.at<double>(1);  odomResetReq.z = SCRChat.at<double>(2);
	odomResetReq.roll = QCRhatroll; odomResetReq.pitch = QCRhatpitch; odomResetReq.yaw = QCRhatyaw; 
}

// -------------------------------------------------------------
/*
resetMap -	Resets map

Author: JDev 161201
	
*/
// -------------------------------------------------------------
void resetMap(){
	// Local variables
	// ..
	
	clearMap = true;
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
	ros::Time begin = ros::Time::now();
	ros::Subscriber odomSub;	// Visual Odometry (VO) subscriber
	ros::Subscriber cloudSub;	// Point cloud subscriber
	ros::Subscriber truePoseSub;
	ros::Subscriber insPoseSub;
	ros::Publisher posePub;
	PoseActionClient PAC(nh, "action/pose");	// Pose action client
	ros::ServiceClient resetMapClient = nh.serviceClient<std_srvs::Empty>("/rtabmap/trigger_new_map");
	ros::ServiceClient resetOdometryClient = nh.serviceClient<rtabmap_ros::ResetPose>("/rtabmap/reset_odom_to_pose");
	std_srvs::Empty emptySrv;

	
	// Local variables
	hector_uav_msgs::PoseGoal poseGoal;		// PoseGoal object for simpleactionclient PoseActionClient

	// Initialisation
	// Subscribe to positionCallBack
	truePoseSub = nh.subscribe("/ground_truth_to_tf/pose", 10, truePositionCallBack);

	// Publish to poseupdate
	posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("poseupdate", 1);

	// Initialise the PoseActionClient
	PAC.waitForServer();
	ROS_INFO("Pose client initialised.");
	
	// Load first submap
	generateWaypoints("pose_0.txt");			// Sets SOLL, TOL
	loadSubmapPCD("submap_0.pcd");
	
	// Spin once for truth data to be obtained.
	// Perform initialisation of variables.
	ros::Rate rate(10.0);
	ROS_INFO("Initialising variables.");
	while(nh.ok() && !initVars()){
		ros::spinOnce();
		rate.sleep();
	}
	
	// Now odom, cloud and pose topics can be subscribed to
	// Subscribe to odom topic
	odomSub = nh.subscribe("/rtabmap/odom", 1, voCallBack);
	// Subscribe to cloud_map topic
	cloudSub = nh.subscribe("rtabmap/cloud_map", 1, cloudCallBack);
	// Subscribe to estimated pose and not true pose
	insPoseSub = nh.subscribe("/pose", 1, insCallBack);
	
	// Loop
	while (nh.ok()){
		// Send current state to posePub (poseupdate topic)
		posePub.publish(gmBLhat);
		// Send current goal to pose client
		poseGoal.target_pose = gmBL_cmd; 
		PAC.sendGoal(poseGoal);
		waypointNav();
		// If VO has been lost for 10 frames or more, reset VO:
		if (VOLossCounter > 10){
			resetVO();
			// Send reset request
			if (!resetOdometryClient.call(odomResetReq, odomResetResp)){
				ROS_INFO("[repeat_node] Failed to reset odometry.");
			} else {
				ROS_INFO("[repeat_node] Succeeded in resetting odometry.");
				// Reset loss counter
				VOLossCounter = 0;
				// Redo ICP upon reset of VO
				doICP = true;
			}
		}
		if (clearMap){
		// (service: trigger_new_map (std_srvs/Empty) )
			if (!resetMapClient.call(emptySrv)){
				ROS_INFO("[repeat_node] Failed to reset map.");
			} else {
				ROS_INFO("[repeat_node] Succeeded in resetting map.");
				clearMap = false;
			}
		}
		ros::spinOnce();
		rate.sleep();
	}

}




