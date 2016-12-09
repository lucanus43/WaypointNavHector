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
	Mat SBLL = Mat::zeros(3,1,CV_64F);
	Mat QBL = Mat::zeros(4,1,CV_64F);
	Mat TCL_cmd;
	Mat TBL_cmd = Mat::zeros(3,3,CV_64F);
	Mat QCL_cmd;
	double wp_radius = 0.1;
	vector<geometry_msgs::Pose> waypointsCL;
	int wp_counter = 1;
	bool doICP = true;
	bool init = true;
	bool next_map = true;
	Mat QCB = Mat::zeros(4,1,CV_64F);
	Mat TCB = Mat::zeros(3,3,CV_64F);
	Mat SCBB = Mat::zeros(3,1,CV_64F);
	Mat QBL_cmd = Mat::zeros(4,1,CV_64F);
	geometry_msgs::PoseStamped gmBL_cmd;
	geometry_msgs::PoseWithCovarianceStamped gmBLhat;
	pcl::PointCloud<pcl::PointXYZ>::Ptr submapCloud (new pcl::PointCloud<pcl::PointXYZ>);
	double QCRhatyaw, QCRhatpitch, QCRhatroll;

// Odometry Variables
	Mat TRLhat = Mat::eye(3,3,CV_64F);
	Mat TCRhat = Mat::zeros(3,3,CV_64F);
	Mat SRLLhat = Mat::zeros(3,1,CV_64F);
	Mat TCLhat= Mat::zeros(3,3,CV_64F);			// Needs initialising
	Mat SCLLhat = Mat::zeros(3,1,CV_64F);		// Needs initialising
	Mat SBLLhat = Mat::zeros(3,1,CV_64F);
	Mat TRL = Mat::zeros(3,3,CV_64F);
	Mat TCL = Mat::zeros(3,3,CV_64F);
	Mat SRLL = Mat::zeros(3,1,CV_64F);
	Mat SCLL = Mat::zeros(3,1,CV_64F);
	Mat TBLhat = Mat::zeros(3,3,CV_64F);
	Mat QBLhat = Mat::zeros(4,1,CV_64F);
	Mat SCRChat = Mat::zeros(3,1,CV_64F);
	Mat QCRhat = Mat::zeros(4,1,CV_64F);
	int VOLossCounter = 0;
	
// Teach node variables
	Mat TOLhat = Mat::zeros(3,3,CV_64F);
	Mat SOLLhat = Mat::zeros(3,1,CV_64F);
	Mat QOLhat = Mat::zeros(4,1,CV_64F);

// Point cloud registration variables
	Mat TROhat = Mat::eye(3,3,CV_64F);
	Mat SORLhat = Mat::zeros(3,1,CV_64F);
	Mat SORRhat = Mat::zeros(3,1,CV_64F);


// Takeoff client (SimpleActionClient from actionlib)
typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseActionClient;

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
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	
	
	ROS_INFO_STREAM("Entering cloudCallBack.");
	// TODO: Do this only once per submap (SRO and TRO should be constant for each cloud).
	if (0){	// DEBUG: SET TO ZERO TODO fix
		// Convert input PointCloud2 to a PointXYZ cloud temp_cloud
		pcl_conversions::toPCL(*input,pcl_pc2);
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
		temp_cloud->is_dense = false;
		// PointXYZ file is in submapCloud
		
		// Perform ICP on temp_cloud
		icp.setInputSource(temp_cloud);
	 	icp.setInputTarget(submapCloud);
	 	
		
		// Align to Final cloud
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);
		
		// DEBUG: Save aligned cloud to file 
		pcl::io::savePCDFileASCII ("pre-aligned.pcd", *temp_cloud);
		pcl::io::savePCDFileASCII ("aligned.pcd", Final);
		
		// ICP results:
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  		icp.getFitnessScore() << std::endl;

		// Obtain error displacement and transformation matrix
		// SRO = error displacement, TRO = error transform
		// TODO: Work out what coordinates SRO is in.
		//cout << "transform after ICP: " << icp.getFinalTransformation() << endl;

		
		// If fitness score is < 0.1, no need to do ICP anymore.
		if (icp.getFitnessScore() < 0.1){
			doICP = false;
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++) {
					TROhat.at<double>(i,j) = icp.getFinalTransformation()(i,j);
				}
				SORRhat.at<double>(i) = icp.getFinalTransformation()(i,3);
			}
			cout << "TROhat: " << TROhat << endl;
			cout << "SORRhat: " << SORRhat << endl;
		}
    }
    
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
	tf::Quaternion tfQCRhat;
	geometry_msgs::Quaternion gmQCRhat;
	
	// Process
	// Obtain SCRC
	SCRChat.at<double>(0) = (odomMsg->pose.pose.position.x);
	SCRChat.at<double>(1) = (odomMsg->pose.pose.position.y);
	SCRChat.at<double>(2) = (odomMsg->pose.pose.position.z);
	
	// Get QCR from quaternion in odomMsg
	QCRhat.at<double>(0) = (odomMsg->pose.pose.orientation.x); 
	QCRhat.at<double>(1) = (odomMsg->pose.pose.orientation.y); 
	QCRhat.at<double>(2) = (odomMsg->pose.pose.orientation.z); 
	QCRhat.at<double>(3) = (odomMsg->pose.pose.orientation.w); 
	
	// If SCRC is zero (VO lost), increment VO loss counter. SBLLhat will not be set to zero.
	// Else, perform VO as per normal.
	if ( fabs(norm(SCRChat)) == 0){
		ROS_INFO_STREAM("VO lost. Incrementing counter.");
		VOLossCounter++;
		// handle VO loss (Update TCLhat, SCLL using IMU)
		TCLhat = TCB*quat2dcm(QBLhat);
		SCLLhat = quat2dcm(QBLhat.t())*TCB.t()*SCBB + SBLLhat;
		ROS_INFO("SCLLhat (IMU): [%f,%f,%f]", SCLLhat.at<double>(0), SCLLhat.at<double>(1), SCLLhat.at<double>(2));
		// Get SCRChat and QCRhat
		SCRChat = TCLhat*SCLLhat - TCLhat*SRLLhat;
		QCRhat = dcm2quat(TCLhat*TRLhat.t());
		ROS_INFO("SCRChat (IMU): [%f,%f,%f]", SCRChat.at<double>(0), SCRChat.at<double>(1), SCRChat.at<double>(2));
		TRLhat = quat2dcm(QCRhat).t()*TCLhat;
		// DEBUG //
		SORLhat = TRLhat.t()*SORRhat;
		// DEBUG //
		ROS_INFO("SORLhat (IMU): [%f,%f,%f]", SORLhat.at<double>(0), SORLhat.at<double>(1), SORLhat.at<double>(2));
		
		//ROS_INFO("SCRChat (from IMU): [%f,%f,%f]", SCRChat.at<double>(0), SCRChat.at<double>(1), SCRChat.at<double>(2));
		
		// Get get camera angles
		gmQCRhat.x = QCRhat.at<double>(0); gmQCRhat.y = QCRhat.at<double>(1); gmQCRhat.z = QCRhat.at<double>(2); gmQCRhat.w = QCRhat.at<double>(3); 
   		tf::quaternionMsgToTF(gmQCRhat, tfQCRhat);
    	tf::Matrix3x3(tfQCRhat).getEulerYPR(QCRhatyaw, QCRhatpitch, QCRhatroll);
		
	} else {
		VOLossCounter = 0;
		ROS_INFO_STREAM("VO Reobtained.");
		//ROS_INFO("SCRChat (VO): [%f,%f,%f]", SCRChat.at<double>(0), SCRChat.at<double>(1), SCRChat.at<double>(2));
		//ROS_INFO_STREAM("QCRhat: " << QCRhat);
		// Convert QCR to TCR
		TCRhat = quat2dcm(QCRhat);
		//ROS_INFO_STREAM("TCRhat: " << TCRhat);
		// Compute SRLLhat and TRLhat - needs to only be done once per submap - constant.
	
		// VO alone approach. This will need TCLhat and SCLLhat to be initialised somehow.
		// e.g. TCLhat = TCB*TBLhat with TBLhat from IMU/magnetometer, SCLLhat = SCBL + SBLLhat with SBLLhat from GPS.
		// TRLhat = TCRhat.t()*TCLhat;	
		// SRLLhat = -TRLhat*SCRChat + SCLLhat;

		// VO + ICP approach:
		TRLhat = TROhat*TOLhat;			// TODO: Get TOLhat embedded into posefiles, get TROhat from ICP
		SRLLhat = -SORLhat + SOLLhat; 	// TODO: Get SROL from ICP (SOLLhat from posefile)
		//ROS_INFO_STREAM("TRLhat: " << TRLhat);
		//ROS_INFO_STREAM("TROhat: " << TROhat);
		//ROS_INFO_STREAM("TOLhat: " << TOLhat);
		// DEBUG //
		SORLhat = TRLhat.t()*SORRhat;
		// DEBUG //
		ROS_INFO("SORLhat (VO): [%f,%f,%f]", SORLhat.at<double>(0), SORLhat.at<double>(1), SORLhat.at<double>(2));
	
		// 'Truth' SRLL and TRL
		TCL = TCB*quat2dcm(QBL);
		TRL = TCRhat.t()*TCL;						// Need TCL
		SCLL = TCL.t()*TCB*SCBB + SBLL;
		SRLL = -TCL.t()*SCRChat + SCLL;				// Need SCLL
		//ROS_INFO_STREAM("TCL (truth): " << TRL);
		//ROS_INFO_STREAM("SRLL (truth): " << SRLL);
	
	
		// Update TCLhat and SCLLhat
		TCLhat = TCRhat*TRLhat;						// TCL = TCR*TRL
		SCLLhat = TCLhat.t()*SCRChat + SRLLhat;		// SCL = SCR + SRL
		ROS_INFO("SCRChat (VO): [%f,%f,%f]", SCRChat.at<double>(0), SCRChat.at<double>(1), SCRChat.at<double>(2));
		ROS_INFO("SRLLhat (VO): [%f,%f,%f]", SRLLhat.at<double>(0), SRLLhat.at<double>(1), SRLLhat.at<double>(2));
		ROS_INFO("SCLLhat (VO): [%f,%f,%f]", SCLLhat.at<double>(0), SCLLhat.at<double>(1), SCLLhat.at<double>(2));
		//ROS_INFO_STREAM("TCRhat : " << TCRhat);
		// Calculate SBLLhat
		SBLLhat = -TCLhat.t()*TCB*SCBB+SCLLhat; 	// SBLL = SBCL + SCLL


		TBLhat = TCB.t()*TCLhat;					// TBL = TBC*TCL
		QBLhat = dcm2quat(TBLhat);
		ROS_INFO_STREAM("QBLhat (VO): " << QBLhat);
		ROS_INFO("SBLLhat mod. (VO): [%f,%f,%f]", SBLLhat.at<double>(0), SBLLhat.at<double>(1), SBLLhat.at<double>(2));
	
	} //endif
	
	

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
	


	//ROS_INFO("Finished odomCallBack");
}


// -------------------------------------------------------------
/*
truePositionCallBack- 	Called when SBLL/TBL is received (listening for TBL - truth)
					Waypoint navigation is performed here.
					
					TODO: Listen for SCLLhat instead -> publish SCLLhat to TF

Author: JDev 161125
	
*/
// -------------------------------------------------------------
void truePositionCallBack( const geometry_msgs::PoseStamped::ConstPtr& gmBL ) {
	// Local variables
	Mat QCLhat;
	geometry_msgs::Pose gmCL;
	
	// Truth 
	SBLL.at<double>(0) = gmBL->pose.position.x;
	SBLL.at<double>(1) = gmBL->pose.position.y;
	SBLL.at<double>(2) = gmBL->pose.position.z;
	QBL.at<double>(0) = gmBL->pose.orientation.x;
	QBL.at<double>(1) = gmBL->pose.orientation.y;
	QBL.at<double>(2) = gmBL->pose.orientation.z;
	QBL.at<double>(3) = gmBL->pose.orientation.w;
	
	// Initialisation
	if (init){	// Set gmBL to truth for initial use.
		ROS_INFO_STREAM("gmBLhat set to truth");
		ROS_INFO("SBLL (truth): [%f,%f,%f]", SBLL.at<double>(0), SBLL.at<double>(1), SBLL.at<double>(2));
		// Set SBLLhat and QBLhat
		SBLLhat = SBLL.clone(); QBLhat = QBL.clone();
		// Set SRLLhat - can be anywhere so long as odom is reset to acknowledge this
		// TODO: Check why true TRL breaks this
		SRLLhat = quat2dcm(QBL).t()*SCBB+SBLL;	// SRLL = initial SCLL
		TRLhat = TCB*quat2dcm(QBLhat);
		// Broadcast truth data as the first estimated state
		gmBLhat.pose.pose.position.x = SBLL.at<double>(0);
		gmBLhat.pose.pose.position.y = SBLL.at<double>(1);
		gmBLhat.pose.pose.position.z = SBLL.at<double>(2);
		gmBLhat.pose.pose.orientation.x = QBL.at<double>(0);
		gmBLhat.pose.pose.orientation.y = QBL.at<double>(1);
		gmBLhat.pose.pose.orientation.z = QBL.at<double>(2);
		gmBLhat.pose.pose.orientation.w = QBL.at<double>(3);
		gmBLhat.pose.covariance[0] = 0.5;
		gmBLhat.pose.covariance[7] = 0.5;
		gmBLhat.pose.covariance[14] = 0.5;
		gmBLhat.pose.covariance[21] = 0.5;
		gmBLhat.pose.covariance[28] = 0.5;
		gmBLhat.pose.covariance[35] = 0.5;
		init = false;
	}
	
	// Force SCLLhat to converge to SCLL_cmd and TCLhat to converge to TCL_cmd
	QCLhat = dcm2quat(TCLhat);
	
	// Output message to user
	ROS_INFO("SBLLhat (at true callback): [%f,%f,%f]", SBLLhat.at<double>(0), SBLLhat.at<double>(1), SBLLhat.at<double>(2));
	ROS_INFO("SBLL_cmd: [%f,%f,%f]", SBLL_cmd.at<double>(0), SBLL_cmd.at<double>(1), SBLL_cmd.at<double>(2));
	//ROS_INFO("SCLLhat: [%f,%f,%f]", SCLLhat.at<double>(0), SCLLhat.at<double>(1), SCLLhat.at<double>(2));
	//ROS_INFO("SCLL_cmd: [%f,%f,%f]", SCLL_cmd.at<double>(0), SCLL_cmd.at<double>(1), SCLL_cmd.at<double>(2));
	//ROS_INFO("SBLL: [%f,%f,%f]", SBLL.at<double>(0), SBLL.at<double>(1), SBLL.at<double>(2));
	//ROS_INFO_STREAM("QCLhat: " << QCLhat);
	//ROS_INFO_STREAM("QCL_cmd: :" << QCL_cmd);
	
	
	// Check to see if waypoints have been reached.
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
					// Convert SCLL_cmd to SBLL_cmd (SBLL = SBCL + SCLL)
					SBLL_cmd = -TCLhat.t()*TCB*SCBB + SCLL_cmd;
					ROS_INFO_STREAM("SBLL_cmd: " << SBLL_cmd);
					TBL_cmd = TCB.t()*quat2dcm(QCL_cmd);
					QBL_cmd = dcm2quat(TBL_cmd);
					// Export commanded pose as gmBL_cmd
					gmBL_cmd.pose.position.x = SBLL_cmd.at<double>(0);
					gmBL_cmd.pose.position.y = SBLL_cmd.at<double>(1);
					gmBL_cmd.pose.position.z = SBLL_cmd.at<double>(2);
					gmBL_cmd.pose.orientation.x = QBL_cmd.at<double>(0);
					gmBL_cmd.pose.orientation.y = QBL_cmd.at<double>(1);
					gmBL_cmd.pose.orientation.z = QBL_cmd.at<double>(2);
					gmBL_cmd.pose.orientation.w = QBL_cmd.at<double>(3);
					
				} else {
					next_map = true;
					ROS_INFO( "Finished the waypoint path!" );
					// DEBUG TODO: get rid of
					ros::shutdown();
					// DEBUG
				}	
			}
		}
	}
}


// -------------------------------------------------------------
/*
positionCallBack- 	Called when SBLLhat/QBLhat is received
					

Author: JDev 161130
	
*/
// -------------------------------------------------------------
void positionCallBack( const geometry_msgs::PoseStamped::ConstPtr& extPose ){
	// Local variables
	tf::Quaternion tfQBLhat;
	
	// State estimate from Hector (IMU + our state published on poseupdate)
	if (!init){
		SBLLhat.at<double>(0) = extPose->pose.position.x;
		SBLLhat.at<double>(1) = extPose->pose.position.y;
		SBLLhat.at<double>(2) = extPose->pose.position.z;
		QBLhat.at<double>(0) = extPose->pose.orientation.x;
		QBLhat.at<double>(1) = extPose->pose.orientation.y;
		QBLhat.at<double>(2) = extPose->pose.orientation.z;
		QBLhat.at<double>(3) = extPose->pose.orientation.w;
	
		TCLhat = TCB*quat2dcm(QBLhat);
		SCLLhat = quat2dcm(QBLhat.t())*TCB.t()*SCBB + SBLLhat;
	
		ROS_INFO("SBLLhat (mod. at positioncallback): [%f,%f,%f]", SBLLhat.at<double>(0), SBLLhat.at<double>(1), SBLLhat.at<double>(2));
	}
}


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
	for (int i = 1; i < vecSCOL.size(); i++){
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
	ros::Subscriber poseSub;
	ros::Publisher posePub;
	PoseActionClient PAC(nh, "action/pose");	// Pose action client
	ros::ServiceClient resetMapClient = nh.serviceClient<std_srvs::Empty>("/rtabmap/trigger_new_map");
	ros::ServiceClient resetOdometryClient = nh.serviceClient<rtabmap_ros::ResetPose>("/rtabmap/reset_odom_to_pose");
	std_srvs::Empty emptySrv;
	rtabmap_ros::ResetPose odomResetSrv;
	rtabmap_ros::ResetPose::Request odomResetReq;
	rtabmap_ros::ResetPose::Response odomResetResp;
	
	
	// Local variables
	hector_uav_msgs::PoseGoal poseGoal;		// PoseGoal object for simpleactionclient PoseActionClient
	hector_uav_msgs::TakeoffGoal takeoffgoal;		// Goal (empty message) for TakeoffClient
	
	
	// Initialisation
	// Subscribe to odom topic
	odomSub = nh.subscribe("/rtabmap/odom", 1, odomCallBack);
	// Subscribe to cloud_map topic
	cloudSub = nh.subscribe("rtabmap/cloud_map", 100, cloudCallBack);
	// Subscribe to positionCallBack
	truePoseSub = nh.subscribe("/ground_truth_to_tf/pose", 1, truePositionCallBack);
	// Subscribe to estimated pose and not true pose
	poseSub = nh.subscribe("/pose", 1, positionCallBack);
	// Publish to poseupdate
	posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("poseupdate", 1);
	
	// Initialise the PoseActionClient
	PAC.waitForServer();
	ROS_INFO("Pose client initialised.");
	
	
	// Start on submap_0 -> pose_0.txt, submap_0.pcd.
	// TODO: Make the map selection based on SCLLhat and SOLL and direction of travel (alternatively,
	// scan all maps and load all SOLL values and compare SCLLhat)
	generateWaypoints("pose_0.txt");
	loadSubmapPCD("submap_0.pcd");
	
	// Set TCB/SCBB
	QCB.at<double>(1) = 0.707; QCB.at<double>(3) = 0.707;
	TCB = quat2dcm(QCB);
	// Set SCBB
	SCBB.at<double>(0) = 0.1;
	SCBB.at<double>(2) = -0.03;
	
	// Set first waypoint goal
	// TODO: use geometry_msgs instead of cv::Mat for commanded pose?
	// TODO: Make a function cv::Mat <- gmBA to save space
	// Set SBLL_cmd to [-1,-1,5] initially
	SCLL_cmd.at<double>(0) = waypointsCL.at(wp_counter).position.x;
	SCLL_cmd.at<double>(1) = waypointsCL.at(wp_counter).position.y;
	SCLL_cmd.at<double>(2) = waypointsCL.at(wp_counter).position.z;
	QCL_cmd.at<double>(0) = waypointsCL.at(wp_counter).orientation.x;
	QCL_cmd.at<double>(1) = waypointsCL.at(wp_counter).orientation.y;
	QCL_cmd.at<double>(2) = waypointsCL.at(wp_counter).orientation.z;
	QCL_cmd.at<double>(3) = waypointsCL.at(wp_counter).orientation.w;
	
	// Convert SCLL_cmd to SBLL_cmd (SBLL = SBCL + SCLL)
	SBLL_cmd = -TCLhat.t()*TCB*SCBB + SCLL_cmd;
	TBL_cmd = TCB.t()*quat2dcm(QCL_cmd);
	QBL_cmd = dcm2quat(TBL_cmd);
	ROS_INFO_STREAM("SCBB: " << SCBB);
	ROS_INFO_STREAM("SBLL_cmd: " << SBLL_cmd);
	ROS_INFO_STREAM("QBL_cmd: " << QBL_cmd);
	
	
	// Export commanded pose as gmBL_cmd
	gmBL_cmd.pose.position.x = SBLL_cmd.at<double>(0);
	gmBL_cmd.pose.position.y = SBLL_cmd.at<double>(1);
	gmBL_cmd.pose.position.z = SBLL_cmd.at<double>(2);
	gmBL_cmd.pose.orientation.x = QBL_cmd.at<double>(0);
	gmBL_cmd.pose.orientation.y = QBL_cmd.at<double>(1);
	gmBL_cmd.pose.orientation.z = QBL_cmd.at<double>(2);
	gmBL_cmd.pose.orientation.w = QBL_cmd.at<double>(3);
	
	
	// Loop
	ros::Rate rate(10.0);
	while(nh.ok()){
		// Waypoint navigation using SCLLhat and SCLL_cmd, TCLhat and TCL_cmd (convert to B frame?)
		//Update our message so the receiving node knows it is recent
		
		gmBL_cmd.header.stamp = ros::Time::now();
		gmBL_cmd.header.seq++;
		gmBL_cmd.header.frame_id = "world";

		gmBLhat.header.stamp = ros::Time::now();
		gmBLhat.header.seq++;
		gmBLhat.header.frame_id = "/nav";			// poseupdate listener does not use header
		
		// Send current state to posePub (poseupdate topic)
		posePub.publish(gmBLhat);
		
		// Send current goal to pose client
		poseGoal.target_pose = gmBL_cmd; 
		PAC.sendGoal(poseGoal);
		
		// If it is the first take off, reset odometry when the first waypoint is reached.
		// OR if VO has been lost for > 10 frames
		if ((init) || VOLossCounter > 10) {
			// reset odometry
			cout << "SBLL: " << SBLL << endl;
			cout << "SBLL_cmd: " << SBLL_cmd << endl;
			// Reset odometry with SCRChat and QCRhat (will be zero on init)
			odomResetReq.x = SCRChat.at<double>(0); odomResetReq.y = SCRChat.at<double>(1);  odomResetReq.z = SCRChat.at<double>(2);
			odomResetReq.roll = QCRhatroll; odomResetReq.pitch = QCRhatpitch; odomResetReq.yaw = QCRhatyaw; 
			if (!resetOdometryClient.call(odomResetReq, odomResetResp)){
				ROS_INFO("[repeat_node] Failed to reset odometry.");
			} else {
				ROS_INFO("[repeat_node] Succeeded in resetting odometry.");
				// Reset loss counter
				VOLossCounter = 0;
				// Redo ICP upon reset of VO
				//doICP = true;
			}
		}
		// Clear map on initialisation or new map alone
		if ((init)) {
		// Set RTABMAP to start new submap (service: trigger_new_map (std_srvs/Empty) )
			if (!resetMapClient.call(emptySrv)){
				ROS_INFO("[repeat_node] Failed to reset map.");
			} else {
				ROS_INFO("[repeat_node] Succeeded in resetting map.");
			}
		}
		
		// if waypoints complete
			// if next submap exists, move to next submap. (set new waypoints/goal)
			// else exit.
		// endif
		
		// Perform spin
		ros::spinOnce();
		rate.sleep();
	}
	
}





















