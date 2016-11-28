/* map_packaging.cpp 
   
Desc:	Reads in a text file with poses (time x,y,z,qx,qy,qz,qw) in an arbitrary frame
		Defines a submap frame and publishes the submap frame to global map frame transform (TSM) along 
		with the displacement of submap frame rel. global map frame (SSM).
		
		Uses CV math library

Author: Jeffrey Devaraj
Date: 161114 

*/

// Includes
#include "cvUtility.h"
#include "map_packaging.h"

// ------------------------- CODE ------------------------------ //

// -------------------------------------------------------------
/*
packageMap - Packages a map (saves displacements and transforms to a text file)
				C: Camera frame
				O: Odometry frame
				L: Local level frame
				
				// TODO: FIX THIS CODE UP. Output is NOT SCAA - output is SCOL

Author: JDev 161108
	
*/
// -------------------------------------------------------------
void packageMap(vector<Mat> vecSCOL, vector<Mat> vecTCL, Mat SOLL, Mat QOL, string poseFileName){	
	// Local variables	
	Mat TCL = Mat::eye(3,3,CV_64F); // Set TCS to eye(3) i.e. S frame aligned to (initial) cam frame.
	Mat SCOL = Mat::zeros(3,1,CV_64F);
	Mat QCL = Mat::zeros(4,1,CV_64F);
	// Use initial camera pose to define submap frame. Based on features seen in first pose?
	
	// Open the output file
	outputFile.open(poseFileName.c_str());
	
	// Save SOLL and QOL
	outputFile << "SOLL: " << SOLL.at<double>(0) << " " << SOLL.at<double>(1) << " " << SOLL.at<double>(2) << endl;
		outputFile << "QOLhat: " << QOL.at<double>(0) << " " << QOL.at<double>(1) << " " << QOL.at<double>(2) << QOL.at<double>(3) << endl;
	// Loop
	// for each pose (SCOL/TCL) stored in vecSCOL/vecTCL: 
	for (int i = 0; i < vecSCOL.size(); i++){
		// Obtain SCAA and TCA from stored vectors
		SCOL = vecSCOL[i];
		//cout << "SCAA: " << SCAA << endl;
		//cout << "vecSCAA[" << i << "]: " << vecSCAA[i] << endl;
		TCL = vecTCL[i];
		// Write SCOL and TCL to file
		// Note: QCL in format qw,qx,qy,qz
		QCL = dcm2quat(TCL);
		outputFile << SCOL.at<double>(0) << " " << SCOL.at<double>(1) << " " << SCOL.at<double>(2) << " " << QCL.at<double>(0) << " " << QCL.at<double>(1) << " " << QCL.at<double>(2) << " " << QCL.at<double>(3) << endl;

	} // end Loop
	
	// Export text file with SBSS and QBSS (quaternion) for the given submap
	outputFile.close();
	
}



bool extractPosesFromFile(fstream &poseFile, Mat &SOLL, Mat &QOL, vector<Mat> &vecSCOL, vector<Mat> &vecQCL){
	// Local variables
	string tempString;
	size_t pos = 0;
	string token;
	int counter = 0;
	string delimiter = " ";
	string discard;
	Mat SCOL = Mat::zeros(3,1,CV_64F);
	Mat QCL = Mat::zeros(4,1,CV_64F);
		
	// Code
	// Get first line (format: SOLL: P1 P2 P3)
	poseFile >> discard >> SOLL.at<double>(0) >> SOLL.at<double>(1) >> SOLL.at<double>(2);
	// Get second line (format: QOLhat QX QY QZ QW):
	poseFile >> discard >> QOL.at<double>(0) >> QOL.at<double>(1) >> QOL.at<double>(2) >> QOL.at<double>(3);

	cout << "SOLL: " << SOLL << endl;
	cout << "SOLL: " << QOL << endl;
	
	// Get line from posefile. Format: SSV - P1 P2 P3 QX QY QZ QW
	while (!poseFile.eof()) {
		poseFile >> SCOL.at<double>(0) >> SCOL.at<double>(1) >> SCOL.at<double>(2) >> QCL.at<double>(0)
		>> QCL.at<double>(1) >> QCL.at<double>(2) >> QCL.at<double>(3);
		// Push back vectors
		vecSCOL.push_back(SCOL.clone());
		vecQCL.push_back(QCL.clone());	
	}
	return true;
}








/*void packageMap(vector<Mat> vecSCAA, vector<Mat> vecTCA, string poseFileName){	
	// Local variables
	Mat TCS = Mat::eye(3,3,CV_64F); // Set TCS to eye(3) i.e. S frame aligned to (initial) cam frame.
	Mat TCA = Mat::zeros(3,3,CV_64F);
	Mat SCAA = Mat::zeros(3,1,CV_64F);
	Mat SCAS = Mat::zeros(3,1,CV_64F);
	Mat SASS = Mat::zeros(3,1,CV_64F);	
	Mat TAS = Mat::zeros(3,3,CV_64F);
	Mat TCM = Mat::zeros(3,3,CV_64F);
	Mat TSM = Mat::zeros(3,3,CV_64F);
	Mat QCA = Mat::zeros(4,1,CV_64F);
	// Use initial camera pose to define submap frame. Based on features seen in first pose?
	
	
	TCA = ang2dcm(quat2eul(vecTCA[0]));
	SCAA = vecSCAA[0];
	SCSS = Mat::zeros(3,1,CV_64F);
	SCAS = TCS.t()*TCA*SCAA;		// SCAA = first pose in pose file, 
	SASS = -SCAS;					// Disp of arbitrary frame wrt. S frame in S frame coords; const.
	TAS = TCA.t();					// TCA = first transform in pose file.
	
	// Calculate transform TSM where S is submap frame and M is global map frame
	// if this is the first submap, then TSM = TSC = eye(3); - i.e. M frame aligned with C0 frame AND S frame.
	// else TSM = TSC*TCM where TCM comes from VO. i.e. TSM = TCM since S aligned with C frame at start of submap.
	
	//TCM = vecTCM[0];				// VO at first timestep (input from teach_node?)
	//TSM = TCS.t()*TCM;
	
	// Open the output file
	outputFile.open(poseFileName.c_str());
	
	// Loop
	// for each pose (Cx) stored in some arbitrary frame SCAA/QCAA: 
	for (int i = 0; i < vecSCAA.size(); i++){
		// SCAA << file
		// TCA << file
		// TCS = TCA*TAS;				// TAS is const.
		// SCAS = TCS.t()*TCA*SCAA;
		// SCSS = SCAS + SASS;
		
		// Obtain SCAA and TCA from stored vectors
		SCAA = vecSCAA[i];
		//cout << "SCAA: " << SCAA << endl;
		//cout << "vecSCAA[" << i << "]: " << vecSCAA[i] << endl;
		TCA = vecTCA[i];
		// Calculate TCS
		TCS = TCA*TAS;		// TAS is constant for each subframe
		// Calculate SCAS
		SCAS = TCS.t()*TCA*SCAA;
		SCSS = SCAS + SASS;		// SASS is constant for each subframe
		
		// Write SCSS and QCS to file
		// Note: QCS in format qw,qx,qy,qz
		QCA = dcm2quat(TCA);
		outputFile << SCAA.at<double>(0) << " " << SCAA.at<double>(1) << " " << SCAA.at<double>(2) << " " << QCA.at<double>(0) << " " << QCA.at<double>(1) << " " << QCA.at<double>(2) << " " << QCA.at<double>(3) << endl;

	} // end Loop
	
	// Export text file with SBSS and QBSS (quaternion) for the given submap
	outputFile.close();
	
}*/




void packageMap(string poseFileLocation, Mat TCM){	
/*	// Local variables
	string discard;
	
	// Read in poses (what frame are they in?)
	poseFile.open(poseFileLocation);
	// Remove first line during initialisation
	getline(poseFile, discard, '\n');
	// Read values into a vector of SCAA and QCA (vecSCAA, vecQCA)
	while (posesAvailable){
		posesAvailable = extractPosesFromFile(poseFile);
	}
	
	// Use initial camera pose to define submap frame. Based on features seen in first pose?
	// Set TCS to eye(3) i.e. S frame aligned to (initial) cam frame.
	TCS = Mat::eye(3,3,CV_64F);
	TCA = quat2dcm(vecQCA[0]);
	SCAA = vecSCAA[0];
	SCSS = Mat::zeros(3,1,CV_64F);
	SCAS = TCS.t()*TCA*SCAA;		// SCAA = first pose in pose file, 
	SASS = -SCAS;					// Disp of arbitrary frame wrt. S frame in S frame coords; const.
	TAS = TCA.t();					// TCA = first transform in pose file.
	
	// Calculate transform TSM where S is submap frame and M is global map frame
	// if this is the first submap, then TSM = TSC = eye(3); - i.e. M frame aligned with C0 frame AND S frame.
	// else TSM = TSC*TCM where TCM comes from VO. i.e. TSM = TCM since S aligned with C frame at start of submap.
	
	//TCM = getTCM();				// VO at first timestep (input from teach_node?)
	TSM = TCS.t()*TCM;
	
	// Open the output file
	outputFile.open("submap_poses.txt");
	
	// Loop
	// for each pose (Cx) stored in some arbitrary frame SCAA/QCAA: 
	for (int i0 = 1; i0 < vecSCAA.size(); i0++){
		// SCAA << file
		// TCA << file
		// TCS = TCA*TAS;				// TAS is const.
		// SCAS = TCS.t()*TCA*SCAA;
		// SCSS = SCAS + SASS;
		
		// Obtain SCAA and TCA from stored vectors
		SCAA = vecSCAA[i0];
		TCA = quat2dcm(vecQCA[i0]);
		// Calculate TCS
		TCS = TCA*TAS;		// TAS is constant for each subframe
		// Calculate SCAS
		SCAS = TCS.t()*TCA*SCAA;
		SCSS = SCAS + SASS;		// SASS is constant for each subframe
		
		// Write SCSS and QCS to file
		// Note: QCS in format qw,qx,qy,qz
		outputFile << SCSS[0] << " " << SCSS[1] << " " << SCSS[2] << " ";
		outputFile << QCS[0] << " " << QCS[1] << " " << QCS[2] << " " << QCS[3] << endl;

	} // end Loop
	
	// Export text file with SBSS and QBSS (quaternion) for the given submap
	outputFile.close();
*/	
}



// Unused code
/*
	if (!poseFile.eof()) {
		getline(poseFile, tempString, '\n');
		//cout << tempString << endl;
		while ((pos = tempString.find(delimiter)) != string::npos) {
			token = tempString.substr(0, pos);
			//cout << token << endl;
			tempString.erase(0, pos + delimiter.length());
			if (counter < 3) {
				// Accelerometer values in string
				double postemp = atof(token.c_str());
				SCOL.at<double>(counter, 0) = postemp;
			}
			else if ((counter > 2) && (counter < 6)) {
				// Gyro values in string
				double quatTemp = atof(token.c_str());
				QCL.at<double>(counter - 3, 0) = quatTemp;
			}
			counter++;
		}
		// Assign final element in string
		QCL.at<double>(counter - 3, 0) = atof(tempString.c_str());
*/
