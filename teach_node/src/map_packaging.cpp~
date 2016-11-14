/* map_packaging.cpp 
   
Desc:	Reads in a point cloud (.pcl) and a text file with poses (time x,y,z,qx,qy,qz,qw)
		Defines a submap frame and publishes the submap frame to global map frame transform (TSM) along 
		with the displacement of submap frame rel. global map frame (SSM).
		
		Uses CV math library

Author: Jeffrey Devaraj
Date: 161114 

*/

// Includes
#include "map_packaging.h"

// ------------------------- CODE ------------------------------ //

void packageMap(string poseFileLocation, Mat TCM){	
	// Local variables
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
	
}



bool extractPosesFromFile(fstream poseFile){
	// Local variables
	string tempString;
	size_t pos = 0;
	string token;
	int counter = 0;
	string delimiter = " ";
	Mat SCAA = Mat::zeros(3,1,CV_64F);
	Mat QCA = Mat::zeros(4,1,CV_64F);
		
	// Code
	// Get line from posefile. Format: SSV - T P1 P2 P3 QX QY QZ QW
	if (!poseFile.eof()) {
		getline(poseFile, tempString, '\n');
		//cout << tempString << endl;
		while ((pos = tempString.find(delimiter)) != string::npos) {
			token = tempString.substr(0, pos);
			//cout << token << endl;
			tempString.erase(0, pos + delimiter.length());
			if (counter > 0 && counter <= 3) {
				// Accelerometer values in string
				double postemp = atof(token.c_str());
				SCAA.at<double>(counter, 0) = postemp;
			}
			else if ((counter > 3) && (counter < 6)) {
				// Gyro values in string
				double quatTemp = atof(token.c_str());
				QCA.at<double>(counter - 3, 0) = quatTemp;
			}
			counter++;
		}
		// Assign final element in string
		QCA.at<double>(counter - 5, 0) = atof(tempString.c_str());
		
		// Push back vectors
		vecSCAA.push_back(SCAA);
		vecQCA.push_back(QBC);
		return true;
	} else { return false; }
}




