/* teach_node.cpp 
   
Desc:	Performs the teach node for visual teach and repeat implementation.
		Note: BG. - Background tasks
		
		TODO: Save poses in discrete intervals or if an intentional pose change is performed.
		TODO: 
		

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
	bool firstTakeOff = true;
	tf::StampedTransform tfTBL;
	tf::StampedTransform tfTCB;
	Mat SBLL_cmd = Mat::zeros(3,1,CV_64F); // TODO: Change convention here for commanded values
	Mat QBL_cmd = Mat::zeros(4,1,CV_64F);	// TODO: Change convention here for commanded values
	double wp_radius = 0.1;
	int fileCounter = 0;
// Map packaging variables
	bool saveMap = false;
	bool next_map = false;
// Truth variables
	Mat SBLL = Mat::zeros(3,1,CV_64F);
	Mat TBL = Mat::zeros(3,3,CV_64F);
	Mat TCL = Mat::zeros(3,3,CV_64F);
	Mat TCB = Mat::zeros(3,3,CV_64F);
	Mat QCB = Mat::zeros(4,1,CV_64F);
	Mat SCBB = Mat::zeros(3,1,CV_64F);
	Mat SCLL = Mat::zeros(3,1,CV_64F);
	Mat QBL = Mat::zeros(4,1,CV_64F);
	Mat QCL = Mat::zeros(4,1,CV_64F);
	Mat SCOL = Mat::zeros(3,1,CV_64F);
	Mat QOL = Mat::zeros(4,1,CV_64F);
	Mat SOLL = Mat::zeros(3,1,CV_64F);
// Odometry variables
	bool firstOdom = true;
	vector<Mat> vecSCOL;
	vector<Mat> vecTCL;
	Mat SBLLhat_vo = Mat::zeros(3,1,CV_64F);
	Mat SCLLhat_vo = Mat::zeros(3,1,CV_64F);
	Mat QBLhat_vo = Mat::zeros(4,1,CV_64F);
	Mat SCOLhat_vo = Mat::zeros(3,1,CV_64F);
	Mat SOLLhat_vo = Mat::zeros(3,1,CV_64F);
	Mat QOLhat_vo = Mat::zeros(4,1,CV_64F);
	Mat TOBhat_vo;
	Mat TOChat_vo;
	Mat TOLhat_vo;
	Mat TCLhat_vo;
// State variables
	Mat SCOChat = Mat::zeros(3,1,CV_64F);
	Mat SCOLhat = Mat::zeros(3,1,CV_64F);
	Mat QCOhat = Mat::zeros(4,1,CV_64F);
	Mat SOLLhat = Mat::zeros(3,1,CV_64F);
	Mat SCLLhat = Mat::zeros(3,1,CV_64F);
	Mat QBLhat = Mat::zeros(4,1,CV_64F);
	Mat SBLLhat = Mat::zeros(3,1,CV_64F);
	Mat QOLhat = Mat::zeros(4,1,CV_64F);
	Mat QCLhat = Mat::zeros(4,1,CV_64F);

// Function prototypes
void updateState(Mat inSBLLhat, Mat inQBLhat, Mat inSOLLhat, Mat inQOLhat);
void initNextSubmap();


// ------------------------- CALLBACK FUNCTIONS ------------------------------ //
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
	
	// SCLL, SRLL
	SCLL = quat2dcm(QBL).t()*SCBB+SBLL;
	QCL = dcm2quat(TCB*quat2dcm(QBL));
	
	
}

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
    stringstream mapfileName;
    stringstream posefileName;
    
    // Notify user of entering function
    //ROS_INFO("[teach_node] Entered cloudCallBack.");
    
    // If saveMap is true, then save map to file and set saveMap = false
    if (saveMap){
    	pcl_conversions::toPCL(*input,pcl_pc2);
    	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    	
    	// Save map files (submap_x.pcd, pose_x.txt)
    	mapfileName.str(std::string());	// Clear stringstream
    	mapfileName << "submap_" << fileCounter << ".pcd";
    	pcl::io::savePCDFileASCII (mapfileName.str().c_str(), *temp_cloud);
    	std::cerr << "Saved " << temp_cloud->points.size() << " data points to " << mapfileName.str() << std::endl;
    	
    	// Save poses here.
    	// Call packageMap and package submap
    	posefileName.str(std::string());	// Clear stringstream
    	posefileName << "pose_" << fileCounter << ".txt";
		cout << "vecSCOL.size(): " << vecSCOL.size() << endl;
		packageMap(vecSCOL, vecTCL, SOLL, QOL, posefileName.str());
		
    	// Set saveMap to false
    	saveMap = false;
    	// Set resetVars to true
    	next_map = true;
    	// Increment fileCounter
    	fileCounter++;
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
	Mat SCOChat_vo = Mat::zeros(3,1,CV_64F);
	Mat QCOhat_vo;
	
	// Process
	// Obtain SCOC
	SCOChat_vo.at<double>(0) = (odomMsg->pose.pose.position.x);
	SCOChat_vo.at<double>(1) = (odomMsg->pose.pose.position.y);
	SCOChat_vo.at<double>(2) = (odomMsg->pose.pose.position.z);
	
	// Get TCO from quaternion in odomMsg
	QCOhat_vo.push_back(odomMsg->pose.pose.orientation.x); 
	QCOhat_vo.push_back(odomMsg->pose.pose.orientation.y); 
	QCOhat_vo.push_back(odomMsg->pose.pose.orientation.z); 
	QCOhat_vo.push_back(odomMsg->pose.pose.orientation.w); 
	
	// If SCRC is zero (VO lost), do not perform VO calculations. SBLLhat will not be set to zero.
	// Else, perform VO as per normal.
	if ( fabs(norm(SCOChat_vo)) == 0){
		ROS_INFO_STREAM("VO lost.");
	} else {
		// Output Quaternion for debugging
		//ROS_INFO("[teach_node] QCO: [%f,%f,%f,%f]", QCOhat.at<double>(0), QCOhat.at<double>(1), QCOhat.at<double>(2), QCOhat.at<double>(3));
	
		// Have TCB (const), TBL (from TF), TCO (from VO ^), TOL (from TCB, TBL), TBL
		TOChat_vo = quat2dcm(QCOhat_vo).t();
		// Obtain TCLhat (estimated TCL from VO)
		TCLhat_vo = TOChat_vo.t()*quat2dcm(QOLhat);		// QOLhat <- from initialisation of submap
		QBLhat_vo = dcm2quat(TCB.t()*TCLhat_vo);

		// Transform SCOC from C coordinates to L coords
		SCOLhat_vo = TCLhat_vo.t()*SCOChat_vo;
		// Calculate SCLLhat (not used, just for output)
		SCLLhat_vo = SCOLhat_vo + SOLLhat;	// SCOLhat <- from initialisation of submap
		SBLLhat_vo = -quat2dcm(QBLhat_vo).t()*SCBB + SCLLhat_vo;
		// Calculate SOLLhat_vo (unused, for display only)
		SOLLhat_vo = -SCOLhat_vo + SCLLhat_vo;

		ROS_INFO_STREAM("[teach_node] SCLL: " << SCLL);
		ROS_INFO_STREAM("[teach_node] SBLL: " << SBLL);
		ROS_INFO_STREAM("[teach_node] SCOLhat_vo: " << SCOLhat_vo);
		ROS_INFO_STREAM("[teach_node] SOLLhat_vo: " << SOLLhat_vo);
		// Output SCOL to WS
		//ROS_INFO("[teach_node] SCOL: [%f,%f,%f]", SCOLhat.at<double>(0), SCOLhat.at<double>(1), SCOLhat.at<double>(2));
	
		// Update state
		updateState(SBLLhat_vo, QBLhat_vo, SOLLhat, QOLhat);
	}
}


// -------------------------------------------------------------
/*
landingCallBack- 	Called when landing action is called.
					Quits main loop.

Author: JDev 161109
	
*/
// -------------------------------------------------------------
void landingCallBack(const hector_uav_msgs::LandingActionGoalConstPtr& landingPose){
	// Set quitImageTransport to true
	//quitTeachNode = true;
	// Destroy view
	ROS_INFO("[teach_node] Waypoints complete. Shutting down.");
	// Exit
	ros::shutdown();
}


// -------------------------------------------------------------
/*
takeoffCallBack- 	Called when takeoff action is called.
					Enables main loop

Author: JDev 161123
	
*/
// -------------------------------------------------------------
void takeoffCallBack(const std_msgs::Empty msg){
	// Set takeOff to true
	ROS_INFO("[teach_node] First takeoff called.");
	firstTakeOff = false;
	// Destroy view
	//ROS_INFO("[teach_node] Waypoints complete. Shutting down.");
	// Exit
	//ros::shutdown();
}


// -------------------------------------------------------------
/*
cmdPoseCallBack- 	Called when a commanded pose is received

Author: JDev 161123
	
*/
// -------------------------------------------------------------
void cmdPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& cmd_pos ){
	// Notify user of function enter
	//ROS_INFO("[teach_node] Cmd pose received.");
	
	// cmd SBL in L frame/coords.
	SBLL_cmd.at<double>(0) = cmd_pos->pose.position.x;
	SBLL_cmd.at<double>(1) = cmd_pos->pose.position.y;
	SBLL_cmd.at<double>(2) = cmd_pos->pose.position.z;
	
	// cmd QBL
	QBL_cmd.at<double>(0) = cmd_pos->pose.orientation.x;
	QBL_cmd.at<double>(1) = cmd_pos->pose.orientation.y;
	QBL_cmd.at<double>(2) = cmd_pos->pose.orientation.z;
	QBL_cmd.at<double>(3) = cmd_pos->pose.orientation.w;
	// When first waypoint is reached set firsTakeOff to false
	//firstTakeOff = false;
}


// ------------------------- PROCESS FUNCTIONS ------------------------------ //

// -------------------------------------------------------------
/*
initVars -	Initialises all variables

Author: JDev 161214
	
*/
// -------------------------------------------------------------
bool initVars(){
	// Local variables
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
	SOLLhat = quat2dcm(QBL).t()*SCBB+SBLL;
	QOLhat = dcm2quat(TCB*quat2dcm(QBL));
	
	// Return true if SBLL is not zero
	if (fabs(norm(SBLL)) > 0){
		// Update state (just in case), use any suffix. All variables are the same.
		updateState(SBLLhat, QBLhat, SOLLhat, QOLhat);	
		ROS_INFO("SOLLhat init: [%f,%f,%f]", SOLLhat.at<double>(0), SOLLhat.at<double>(1), SOLLhat.at<double>(2));
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

void updateState(Mat inSBLLhat, Mat inQBLhat, Mat inSOLLhat, Mat inQOLhat){
	// Local Variables
	// ..
	
	// Average current state with input state
	SBLLhat = 0.5*(SBLLhat + inSBLLhat);
	QBLhat = 0.5*(QBLhat + inQBLhat);
	SOLLhat = 0.5*(SOLLhat + inSOLLhat);
	QOLhat = 0.5*(QOLhat + inQOLhat);
	
	// Normalise updated quaternions
	QBLhat = quatnormalise(QBLhat);
	QOLhat = quatnormalise(QOLhat);
	
	// Update all state variables with new SBLLhat, QBLhat
	SCLLhat = quat2dcm(QBLhat).t()*SCBB + SBLLhat;
	SCOLhat = SCLLhat - SOLLhat;
	// ..
	QCLhat = dcm2quat(TCB*quat2dcm(QBLhat));
	QCOhat = dcm2quat(quat2dcm(QCLhat)*quat2dcm(QOLhat).t());
	//..
	SCOChat = quat2dcm(QCLhat)*SCOLhat;		// Unused for now. Will be used when INS added.
	
	
	// Truth variables (debug)
	SCOL = SCLL - SOLL;
	
	ROS_INFO("SCOLhat: [%f,%f,%f]", SCOLhat.at<double>(0), SCOLhat.at<double>(1), SCOLhat.at<double>(2));
	
	// Push poses back into vectors
	// TODO: Only save poses when a certain distance has been travelled or angular change has been observed.
	// For now only save SCOL and TCL (camera pose wrt. local level frame).
	// TRUTH VARIABLES - DEBUG
	vecSCOL.push_back(SCOL.clone());
	vecTCL.push_back(quat2dcm(QCL).clone());
}


// -------------------------------------------------------------
/*
initNextMap -	Initialises the next submap

Author: JDev 161214
	
*/
// -------------------------------------------------------------
void initNextSubmap(){
	// Local variables
	// ..
	
	// Reset vecSCOL, vecTCL, SCOLhat nad TCLhat.
	vecSCOL.clear();
	vecTCL.clear();
	// Reset SOLLhat, QOLhat to reflect new O frame
	SOLLhat = SCLLhat.clone();
	SOLLhat_vo = SOLLhat.clone();
	// QRLhat reset
	QOLhat = QCLhat.clone();
	QOLhat_vo = QOLhat.clone();
	// Set SCOChat to zero
	SCOChat = Mat::zeros(3,1,CV_64F);
	// update state
	updateState(SBLLhat, QBLhat, SOLLhat, QOLhat);	
	
	// Set truth variables
	SOLL = SCLL.clone();
	QOL = QCL.clone();
	
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
	ros::Subscriber cloudSub;		// Point cloud subscriber
	ros::Subscriber landSub;		// Landing subscriber
	ros::Subscriber takeOffSub; 	// TakeOff subscriber
	ros::Subscriber cmdPoseSub;		// Commanded pose subscriber
	ros::Subscriber odomSub;		// Visual Odometry (VO) subscriber
	ros::Subscriber truePoseSub; 	// True position subscriber
	tf::TransformListener TBLlistener;
	//tf::TransformListener TCBlistener;
	
	// Local variables
	ros::ServiceClient resetMapClient = nh.serviceClient<std_srvs::Empty>("/rtabmap/reset");
	ros::ServiceClient resetOdometryClient = nh.serviceClient<std_srvs::Empty>("/rtabmap/reset_odom");
	std_srvs::Empty emptySrv;
	
	// Process:
	// INITIALISATION SUBSCRIPTIONS
	// Subscribe to true position
	truePoseSub = nh.subscribe("/ground_truth_to_tf/pose", 10, truePositionCallBack);
	
	// Initialisation loop
	ros::Rate rate(20.0);
	// Loop (till variables are initialised)
	while(nh.ok() && !initVars()){
		ros::spinOnce();
		rate.sleep();
	}
	
	// Notify user of initialisation
	ROS_INFO("[teach_node] teach_node initialised. Entering loop.");
	
	
	// POST-INITIALISATION SUBSCRIPTIONS
	// Subscribe to landing action topic
	landSub = nh.subscribe("action/landing/goal", 1, landingCallBack);
	// Subscribe to takeoff action topic
	takeOffSub = nh.subscribe("action/takeoff", 1, takeoffCallBack);
	// Command Pose subscriber
	cmdPoseSub = nh.subscribe("/command/pose", 1, cmdPoseCallBack);
	// Subscribe to odom topic
	odomSub = nh.subscribe("/rtabmap/odom", 1, odomCallBack);
	// Subscribe to cloud_map topic
	cloudSub = nh.subscribe("rtabmap/cloud_map", 1, cloudCallBack);
	
	
	// Main Loop (till waypoints are complete/landing initiated)
	while(nh.ok()){
		// For now, reset odometry when the first waypoint is reached (takeoff)
		
		// Check to see if we need a new map
		if (!saveMap && (fabs(norm(SCOL)) > 2.0)){		// |SCOL| > 2 m and map isn't currently being saved			// Save current point cloud map from cloud_map topic to pcd file
			// if listening to cloud_map topic, set a flag for cb function to save.
			ROS_INFO("[teach_node] Saving map to file.");
			ROS_INFO_STREAM("[teach_node] (fabs(norm(SCOLhat)): " << (fabs(norm(SCOLhat))));
			saveMap = true;
		}// Map will be reset on next call to cloudCallBack
		
		if (next_map){
			// initialise new map
    		initNextSubmap();
			next_map = false;
			saveMap = false;
			// Reset odom and map
			if (!resetOdometryClient.call(emptySrv)){
				ROS_INFO("[teach_node] Failed to reset odometry.");
			} else {
				ROS_INFO("[teach_node] Succeeded in resetting odometry.");
			}
			// Set RTABMAP to start new submap (service: rtabmap/reset (std_srvs/Empty) )
			if (!resetMapClient.call(emptySrv)){
				ROS_INFO("[teach_node] Failed to reset map.");
			} else {
				ROS_INFO("[teach_node] Succeeded in resetting map.");
			}
		}// endif
		
		
		// TRUTH DATA USED HERE. 
		// TODO: Use estimated poses instead of truth data
		// TODO: Perform waypoint navigation inside teach_node -> When first waypoint reached, start saving poses.
		if (firstTakeOff && fabs(norm(SBLL_cmd)) > 0.0) {
			if( fabs( SBLL.at<double>(0) -  SBLL_cmd.at<double>(0) ) < wp_radius && fabs( QBL.at<double>(0) -  QBL_cmd.at<double>(0) ) < wp_radius) {
				if( fabs( SBLL.at<double>(1) - SBLL_cmd.at<double>(1))  < wp_radius && fabs( QBL.at<double>(1) -  QBL_cmd.at<double>(1) ) < wp_radius) {
					if( fabs( SBLL.at<double>(2) - SBLL_cmd.at<double>(2) )  < wp_radius && fabs( QBL.at<double>(2) -  QBL_cmd.at<double>(2) ) < wp_radius) {
						// reset odometry
						cout << "SBLL: " << SBLL << endl;
						cout << "SBLL_cmd: " << SBLL_cmd << endl;
						if (!resetOdometryClient.call(emptySrv)){
							ROS_INFO("[teach_node] Failed to reset odometry.");
						} else {
							ROS_INFO("[teach_node] Succeeded in resetting odometry.");
							firstTakeOff = false;
						}
						// firstTakeOff set to false when takeoffcallback is called.
						// Reset variables
						next_map = true;
					}
				}
			}
		}
		
		
		// TODO: Use VOLossCounter to reset VO using INS. Use reset to pose (SCOChat).
		

		ros::spinOnce();
		rate.sleep();
	}// end loop
} // end main






