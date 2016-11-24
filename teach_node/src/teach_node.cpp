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
	Mat SBLL = Mat::zeros(3,1,CV_64F);
	Mat SBLL_cmd = Mat::zeros(3,1,CV_64F); // TODO: Change convention here for commanded values
	Mat QBL_cmd = Mat::zeros(4,1,CV_64F);	// TODO: Change convention here for commanded values
	double wp_radius = 0.01;
	int fileCounter = 0;
// Map packaging variables
	bool saveMap = false;
	bool resetVars = false;
// Odometry variables
	bool firstOdom = true;
	vector<Mat> vecSCOL;
	vector<Mat> vecTCL;
	Mat SCOLhat = Mat::zeros(3,1,CV_64F);
	Mat TCL = Mat::zeros(3,3,CV_64F);
	Mat TBL;
	Mat TOBhat;
	Mat TCB;
	Mat TOChat;
	Mat TOLhat;
	Mat TCLhat;
	Mat QBL = Mat::zeros(4,1,CV_64F);
	Mat QCB;
	Mat SCBB = Mat::zeros(3,1,CV_64F);
	Mat SCLL;
	Mat SOLLhat = Mat::zeros(3,1,CV_64F);
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
    stringstream mapfileName;
    stringstream posefileName;
    
    // Notify user of entering function
    //ROS_INFO("[teach_node] Entered cloudCallBack.");
    
    // If saveMap is true, then save map to file and set saveMap = false
    if (saveMap){
    	pcl_conversions::toPCL(*input,pcl_pc2);
    	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    	
    	// TODO: Update file naming
    	mapfileName.str(std::string());	// Clear stringstream
    	mapfileName << "submap_" << fileCounter << ".pcd";
    	pcl::io::savePCDFileASCII (mapfileName.str().c_str(), *temp_cloud);
    	std::cerr << "Saved " << temp_cloud->points.size() << " data points to " << mapfileName.str() << std::endl;
    	// Save map here.
    	//..
    	
    	// Save poses here.
    	// Call packageMap and package submap
    	posefileName.str(std::string());	// Clear stringstream
    	posefileName << "pose_" << fileCounter << ".txt";
		cout << "vecSCOL.size(): " << vecSCOL.size() << endl;
		packageMap(vecSCOL, vecTCL, SOLLhat, posefileName.str());
    	
    	// Set saveMap to false
    	saveMap = false;
    	// Set resetVars to true
    	resetVars = true;
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
	Mat SCOChat = Mat::zeros(3,1,CV_64F);
	Mat QCOhat;
	tf::Quaternion tfQBL;
	tf::Quaternion tfQCB;
	geometry_msgs::Quaternion gmQBL;
	geometry_msgs::Quaternion gmQCB;
	
	
	// Initialisation - perform on first callback
	if (firstOdom){
		// Obtain TCB (static transform, QCB = [0.000, 0.707, 0.000, 0.707])
		// TODO: Work out why camera_link to base_link is shown in rqt_tf_tree but lookupTransform() fails?
		QCB = Mat::zeros(4,1,CV_64F);
		QCB.at<double>(1) = 0.707; QCB.at<double>(3) = 0.707;
		TCB = quat2dcm(QCB);
		
		// Set SCBB
		SCBB.at<double>(0) = 0.1;
		SCBB.at<double>(2) = -0.03;
		
		// Set firstOdom to false
		firstOdom = false;
	}
	
	// Process
	// Obtain SCOC
	SCOChat.at<double>(0) = (odomMsg->pose.pose.position.x);
	SCOChat.at<double>(1) = (odomMsg->pose.pose.position.y);
	SCOChat.at<double>(2) = (odomMsg->pose.pose.position.z);
	
	// Notify user that CB functon has been entered
	ROS_INFO("[teach_node] SCOC: [%f,%f,%f]", SCOChat.at<double>(0), SCOChat.at<double>(1), SCOChat.at<double>(2));
	
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
	
	TCL = TCB*TBL; 					// Truth
	
	
	// DEBUG OUTPUTS //
	//cout << "TCLhat: " << TCLhat << endl;
	//cout << "TCL: " << TCL << endl;
	//cout << "TOChat: " << TOChat << endl;
	//cout << "TCB: " << TCB << endl;
	//cout << "TOChat*TCB: " << TOChat*TCB << endl;
	//cout << "TOBhat: " << TOBhat << endl;
	//cout << "TOLhat: " << TOLhat << endl;
	//cout << "TBL: " << TBL << endl;
	// DEBUG OUTPUTS //

	
	// TODO: Convert QCM to a tf::quaternion and set TCL as a tf::matrix3x3
		// TCL = tf::matrix3x3(odomMsg->pose.pose.orientation)?
	
	// TODO: Get SOLL for map.
	
	
	
	// Transform SCOC from C coordinates to L coords
	SCOLhat = TCLhat.t()*SCOChat;
	
	// Obtain SOLLhat
	// Calculate and embed SOLL in output file. This variable will be constant for each submap.
	SCLL = TBL.t()*SCBB + SBLL;			// SCLL = SCBL + SBLL -> truth (SCBB/SBLL known)
	SOLLhat = -SCOLhat + SCLL;			// SCOL is SCOL?
	
	ROS_INFO_STREAM("[teach_node] SCLL: " << SCLL);
	ROS_INFO_STREAM("[teach_node] SBLL: " << SBLL);
	ROS_INFO_STREAM("[teach_node] SCOLhat: " << SCOLhat);
	ROS_INFO_STREAM("[teach_node] SOLLhat: " << SOLLhat);
	// Output SCOL to WS
	//ROS_INFO("[teach_node] SCOL: [%f,%f,%f]", SCOLhat.at<double>(0), SCOLhat.at<double>(1), SCOLhat.at<double>(2));
	
	// Push poses back into vectors
	// TODO: Only save poses when a certain distance has been travelled or angular change has been observed.
	// For now only save SCOL and TCL (camera pose wrt. local level frame).
	//cout << "SCOLhat: " << SCOLhat << endl;
	vecSCOL.push_back(SCOLhat.clone());
	vecTCL.push_back(TCLhat.clone());
	
	// TODO: why is vecSCOL getting the same SCOLhat value pushed to all of it?
	// debug - output vecSCOL to screen
		//cout << "vecSCOL[" << vecSCOL.size()-1 << "]: " << vecSCOL[vecSCOL.size()-1] << endl;
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
			TODO: Use standard callback for generic message
				takeoffCallBack(const std_msgs::Empty msg)

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
	ros::Subscriber takeOffSub; // TakeOff subscriber
	ros::Subscriber cmdPoseSub;	// Commanded pose subscriber
	ros::Subscriber odomSub;	// Visual Odometry (VO) subscriber
	tf::TransformListener TBLlistener;
	//tf::TransformListener TCBlistener;
	
	// Local variables
	Mat oldSCOL = Mat::zeros(3,1,CV_64F);
	
	
	ros::ServiceClient resetMapClient = nh.serviceClient<std_srvs::Empty>("/rtabmap/trigger_new_map");
	ros::ServiceClient resetOdometryClient = nh.serviceClient<std_srvs::Empty>("/rtabmap/reset_odom");
	std_srvs::Empty emptySrv;
	
	// Process:
	
	// Initialisation:
	// Subscribe to landing action topic
	landSub = nh.subscribe("action/landing/goal", 1000, landingCallBack);
	// Subscribe to takeoff action topic
	takeOffSub = nh.subscribe("action/takeoff/status", 1000, takeoffCallBack);
	// Command Pose subscriber
	cmdPoseSub = nh.subscribe("/command/pose", 1000, cmdPoseCallBack);
	// Subscribe to odom topic
	odomSub = nh.subscribe("/rtabmap/odom", 1, odomCallBack);
	// Subscribe to cloud_map topic
	cloudSub = nh.subscribe("rtabmap/cloud_map", 1000, cloudCallBack);
	
	// Obtain initial SCOL from VO (will be 0) -> oldSCOL
	oldSCOL = SCOLhat;
	
	// Notify user of initialisation
	ROS_INFO("[teach_node] teach_node initialised. Entering loop.");
	
	ros::Rate rate(10.0);
	// Loop (till waypoints are complete/landing initiated)
	while(nh.ok()){
		// BG. Perform Waypoint navigation. (Build a launch file that launches waypointnav) //
		// BG. Rtabmap point cloud building in background (rosspawn? Need VO) //
		
		// BG. Obtain TCL from VO(RTABMAP publishes VO to /odom topic) //
		// BG. Obtain SCOL from VO	//
		
		
		
		// TODO: Check if VO is lost and reset VO?
		
		// TODO: Waypoint navigation in teach node.
		// TODO: For now, reset odometry when the first waypoint is reached (takeoff)
		if (firstTakeOff && fabs(norm(SBLL_cmd)) > 0.0) {
			if( fabs( SBLL.at<double>(0) -  SBLL_cmd.at<double>(0) ) < wp_radius && fabs( QBL.at<double>(0) -  QBL_cmd.at<double>(0) ) < wp_radius) {
				if( fabs( SBLL.at<double>(1) - SBLL_cmd.at<double>(1))  < wp_radius && fabs( QBL.at<double>(1) -  QBL_cmd.at<double>(1) ) < wp_radius) {
					if( fabs( SBLL_cmd.at<double>(2) - SBLL_cmd.at<double>(2) )  < wp_radius && fabs( QBL.at<double>(2) -  QBL_cmd.at<double>(2) ) < wp_radius) {
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
						resetVars = true;
					}
				}
			}
		}
		// TODO: Work out why this does not work:
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
			TBLlistener.waitForTransform("world", "base_link", ros::Time::now(), ros::Duration(0.5));
			TBLlistener.lookupTransform("world", "base_link", ros::Time(0), tfTBL);
			// Set SBLL
			SBLL.at<double>(0) = tfTBL.getOrigin().getX(); 
			SBLL.at<double>(1) = tfTBL.getOrigin().getY(); 
			SBLL.at<double>(2) = tfTBL.getOrigin().getZ();
		} catch (tf2::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
     		//ros::Duration(1.0).sleep();
      		//continue;
		}
	
		
		// Check to see if we need a new map
		// TODO: Check if this condition works
		if (!saveMap && (fabs(norm(SCOLhat)) > 2.0)){		// |SCOL| > 2 m and map isn't currently being saved			// Save current point cloud map from cloud_map topic to pcd file
			// if listening to cloud_map topic, set a flag for cb function to save.
			ROS_INFO("[teach_node] Saving map to file.");
			ROS_INFO_STREAM("[teach_node] (fabs(norm(SCOLhat)): " << (fabs(norm(SCOLhat))));
			saveMap = true;
			// TODO: Save map before resetting (reset on next iteration)
		}// Map will be reset on next call to cloudCallBack
		if (resetVars){
			// Reset vecSCOL and vecTCL
			ROS_INFO("[teach_node] Resetting variables.");
			vecSCOL.clear();
			vecTCL.clear();
			// Reset SCOL and TCL;
			SCOLhat.release();
			TCLhat.release();
			// Reset SCOChat and TOChat
			// Reset odometry
			if (!resetOdometryClient.call(emptySrv)){
				ROS_INFO("[teach_node] Failed to reset odometry.");
			} else {
				ROS_INFO("[teach_node] Succeeded in resetting dometry.");
			}
			// Set RTABMAP to start new submap (service: trigger_new_map (std_srvs/Empty) )
			if (!resetMapClient.call(emptySrv)){
				ROS_INFO("[teach_node] Failed to reset map.");
			} else {
				ROS_INFO("[teach_node] Succeeded in resetting map.");
			}
			resetVars = false;
			saveMap = false;
			// Sleep 0.5 seconds since it takes this long for the odometry to reset.
			ros::Duration(0.5).sleep();
		}// endif
		ros::spinOnce();
		rate.sleep();
	}// end loop
} // end main






