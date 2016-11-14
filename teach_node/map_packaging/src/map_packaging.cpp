/* map_packacing.cpp 
   
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
	
	// Read in point cloud
	
	// Read in poses (what frame are they in?)
	poseFile.open(poseFileLocation);
	// Read values into a vector of SCAA and QCA
	// Remove first line during initialisation
	getline(poseFile, discard, '\n');
	
	while (posesAvailable){
		posesAvailable = extractPosesFromFile(poseFile);
	}
	
	// Use initial camera pose to define submap frame. Based on features seen in first pose?
	// Set TCS to eye(3) i.e. S frame aligned to (initial) cam frame.
	TCS = Mat::eye(3); 
	SCSS = Mat::zeros(3,1,CV_64F);
	SCAS = TCS.t()*TCA*SCAA;		// SCAA = first pose in pose file, 
	SASS = -SCAS;					// Disp of arbitrary frame wrt. S frame in S frame coords; const.
	TAS = TCA.t();					// TCA = first transform in pose file.
	
	// Calculate transform TSM where S is submap frame and M is global map frame
	// if this is the first submap, then TSM = TSC = eye(3); - i.e. M frame aligned with C0 frame AND S frame.
	// else TSM = TSC*TCM where TCM comes from VO. i.e. TSM = TCM since S aligned with C frame at start of submap.
	
	//TCM = getTCM();				// VO at first timestep (input from teach_node?)
	TSM = TCS.t()*TCM;
	// Loop
	// for each pose (Cx) stored in some arbitrary frame SCAA/QCAA: 
		// SCAA << file
		// TCA << file
		// TCS = TCA*TAS;				// TAS is const.
		// SCAS = TCS.t()*TCA*SCAA;
		// SCSS = SCAS + SASS;
		// 
		
	
		
		
	
		// Calculate SSM 
	
		// Calculate SBSS from input poses and TSM(?)
	
	// end Loop
	
	// Export text file with SBSS and QBSS (quaternion) for the given submap
	
	// 
	
}



bool extractPosesFromFile(fstream poseFile){
	// Local variables
	string tempString;
	size_t pos = 0;
	string token;
	int counter = 0;
	string delimiter = " ";
	Mat SCAA = Mat::zeros(3,1,CV_64F);
	Mat QBC = Mat::zeros(4,1,CV_64F);
		
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
				QBC.at<double>(counter - 3, 0) = quatTemp;
			}
			counter++;
		}
		// Assign final element in string
		QBC.at<double>(counter - 5, 0) = atof(tempString.c_str());
		
		// Push back vectors
		vecSCAA.push_back(SCAA);
		vecQBC.push_back(QBC);
		return true;
	} else { return false; }
}




