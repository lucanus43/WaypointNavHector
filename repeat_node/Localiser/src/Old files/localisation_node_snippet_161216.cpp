// Local variables
	Mat errSORO = Mat::zeros(3,1,CV_64F);
	Mat errQOR = Mat::zeros(4,1,CV_64F);
	Mat errTOR = Mat::zeros(3,3,CV_64F);
	
	// Read in gmerrOR to errSORO and errTOR;
	errSORO.at<double>(0) = gmerrOR->pose.position.x;
	errSORO.at<double>(1) = gmerrOR->pose.position.y;
	errSORO.at<double>(2) = gmerrOR->pose.position.z;
	errQOR.at<double>(0) = gmerrOR->pose.orientation.x;
	errQOR.at<double>(1) = gmerrOR->pose.orientation.y;
	errQOR.at<double>(2) = gmerrOR->pose.orientation.z;
	errQOR.at<double>(3) = gmerrOR->pose.orientation.w;
	
	// Normalise errQOR
	errQOR = quatnormalise(errQOR);
	
	// Process
	if (1){
		// Convert errQOR to errTOR
		errTOR = quat2dcm(errQOR).t();
		// Calculate SORRhat_icp, TROhat_icp
		TORhat_icp = errTOR.t()*quat2dcm(QROhat).t();				// QROhat <- state update
		cout << "QORhat_icp: " << dcm2quat(TORhat_icp) << endl;
		cout << "QORhat: " << dcm2quat(quat2dcm(QROhat).t()) << endl;
		
		SORRhat_icp = -TORhat_icp.t()*errSORO - SRORhat;			// SORRhat <- state update
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
		ROS_INFO("QBLhat_icp: [%f,%f,%f,%f]", QBLhat_icp.at<double>(0), QBLhat_icp.at<double>(1), QBLhat_icp.at<double>(2), QBLhat_icp.at<double>(3));
		ROS_INFO("QBL: [%f,%f,%f,%f]", QBL.at<double>(0), QBL.at<double>(1), QBL.at<double>(2), QBL.at<double>(3));
		// Perform state update
		updateState(SBLLhat_icp, QBLhat_icp, SRLLhat_icp, QRLhat_icp);
		resetMap();

