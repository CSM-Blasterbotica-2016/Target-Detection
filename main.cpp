// Find and track a textured planar object.
#include "includes.h"
#include "FeatureFinder.h"
#include "Serial.h"

using namespace std;

int main(int argc, char* argv[]) {
	//////////////////////////////////////////
	//Configuration Values
	int vidInd = 0;
	string cameraFile = "robocam_calib.xml";
	string boardFile = "hybrid_aruco.xml";
	char* comport = "COM5";
	//////////////////////////////////////////

	//////////////////////////////////////////
	//Constants
	const int FIELD_LENGTH = 7380;
	const int FIELD_WIDTH = 3750;
	//////////////////////////////////////////
	
	
	cv::Mat videoFrame;
	cv::Mat grayscaleFrame;
	vector<cv::Point2f> features;

	cv::VideoCapture video(vidInd);
	if (!video.isOpened()){
		cout << "Could not access Camera\n";
		exit(1);
	}

	cv::Mat cameraMatrix, distCoeffs, map1, map2;
	cv::FileStorage fs(cameraFile, cv::FileStorage::READ);
	if (!fs.isOpened()){
		std::cout << "Could not read camera parameters\n";
		exit(1);
	}

	cv::Size size;
	//Loads camera paramters
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	fs["image_Height"] >> size.height;
	fs["image_Width"] >> size.width;
	fs.release();

	//Allows undistortion of camera
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
		getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, size, 1, size, 0),
		size, CV_16SC2, map1, map2);

	//Starts feature finder with desired toarget


	FeatureFinder Target(boardFile, cameraFile);
	// Checks if there should be a sub pattern file
	bool isMultiBoard = false;
	string altBoardFile;
	cv::FileStorage bf(boardFile, cv::FileStorage::READ);

	if (!bf.isOpened()){
		std::cout << "Could not read board parameters\n";
		exit(1);
	}
	bf["childBoard"] >> altBoardFile;

	FeatureFinder altTarget;
	if (altBoardFile.length() > 0) {
		isMultiBoard = true;
		altTarget = FeatureFinder(altBoardFile, cameraFile);
	}


	

	
	//Set camera position, change x, y, z. Based on center of camera
	Location center;
	cv::Mat rvec, tvec;
	vector<Location> locs;

	float theta = 0, phi = 0;
	Location robotLoc(0, 0, 0, 0, 0, 0);
	Location cameraLoc(0, 0, 0, 0, 0, 0);
	Location boardLoc;
	cameraLoc.setAngles(phi, theta, 0);

	//Creates serial communication with robot
	Serial superserial = Serial(comport);

	if (!superserial.IsConnected()) {
		cout << "Cannot access arduino, no tracking enabled" << endl;
	}
	else {
		//Starts camera at Theta = 0 Phi = 0
		char initdata[50];
		sprintf_s(initdata, "T%iP%i\n", (int)(theta * 1000), (int)(phi * 1000));
		cout << initdata << endl;
		superserial.WriteData(initdata, 50);
	}
	
	bool running = true;
	bool pause = false;
	bool lost = true;

	//Main loop
	while (running) {
		//Create illustration
		cv::Mat Field(FIELD_LENGTH / 10, FIELD_WIDTH / 10, CV_8UC3);
		Field.setTo(cv::Scalar(0, 0, 0));

		//Read video
		video.read(videoFrame); //Clears frame buffer, gets new frame
		video.read(videoFrame);

		cv::Mat vidframe2 = videoFrame.clone();

		cv::Rect PredictRect; //Unimplimented (for now)

		//Finds patterns
		center = Target.findPattern(vidframe2, PredictRect);
		lost = true;
		if (Target.wasSuccessful()){
			lost = false;
		} else {
			if (isMultiBoard) {
				center = altTarget.findPattern(vidframe2, PredictRect);
				if (altTarget.wasSuccessful()){
					lost = false;
				}
			}
		}

		if (!lost){ // z=0 means target inside camera, impossible
			// Finds target	
			cv::Mat boardTVec;
			cv::Mat boardRVec;
			cv::composeRT(center.getRvec(), center.getTvec(), cameraLoc.getRvec(), cameraLoc.getTvec(), boardRVec, boardTVec);
			boardLoc = Location(boardRVec, boardTVec);
			robotLoc = Location(boardLoc.getTF().inv());
			cout << robotLoc.getLoc() << endl;

			//Adds location to saved location
			locs.push_back(robotLoc);

			//Outputs location to console (for debugging)
			cout << robotLoc.getLoc() << endl;

			float centx = 0, centy = 0;

			vector<cv::Point3f> targcenter;
			targcenter.push_back(cv::Point3f(0, 0, 0));
			vector <cv::Point2f> piccenter;

			cv::projectPoints(targcenter, center.getRvec(), center.getTvec(), cameraMatrix, distCoeffs, piccenter);

			centx = piccenter.at(0).x;
			centy = piccenter.at(0).y;

			cv::circle(videoFrame, piccenter.at(0), 5, cv::Scalar(255, 0, 0), 1);

			/*for (int i = 0; i < features.size(); i++){
				centx += features.at(i).x;
				centy += features.at(i).y;
			}
			centx *= 1 / ((double)features.size());
			centy *= 1 / ((double)features.size());*/

			cout << "x: " << centx << "y: " << centy << endl;

			cout << atan2((centx - cameraMatrix.at<double>(0, 2)), cameraMatrix.at<double>(0, 0)) << endl;

			if (superserial.IsConnected()){
				float dtheta = atan2((centx - cameraMatrix.at<double>(0, 2)), cameraMatrix.at<double>(0, 0));
				float dphi = atan2((centy - cameraMatrix.at<double>(1, 2)), cameraMatrix.at<double>(1, 1));

				//Calculates theta and phi
				/*float dist = sqrt(x*x + y*y + z*z);
				float theta = atan2((float)z, x);
				float phi = asin(((float)y) / dist);
				*/

				if (abs(dtheta) > PI / 32) {
					theta -= dtheta;
				}

				if (abs(dphi) > PI / 32) {
					phi -= dphi;
				}

				//Bounds theta and phi to -90 - 90
				if (theta < -PI / 2) {
					theta = -PI / 2;
				}
				else if (theta > PI / 2){
					theta = PI / 2;
				}

				if (phi < -PI / 2) {
					phi = -PI / 2;
				}
				else if (phi > PI / 2){
					phi = PI / 2;
				}

				//Writes sero data to arduino
				char data[50];
				sprintf_s(data, "T%iP%i\n", (int)(-theta * 1000), (int)(phi * 1000));
				bool result = superserial.WriteData(data, 50);
				cout << data << endl;

				//Delays extra to give servo time to respond
				cvWaitKey(100);
			}			
		}		



		// Outputs path to screen
		for (int i = 1; i < locs.size(); i++){
			cv::Point Point1 = cv::Point((locs.at(i).getX() + FIELD_WIDTH / 2) / 10, (locs.at(i).getZ()) / 10);
			cv::Point Point2 = cv::Point((locs.at(i - 1).getX() + FIELD_WIDTH / 2) / 10, (locs.at(i - 1).getZ()) / 10);
			line(Field, Point1, Point2, cv::Scalar(0, 0, 255));
			circle(Field, Point2, 2, cv::Scalar(255, 255, 255), 0);
		}

		// Outputs last location as green dot
		if (locs.size() != 0) {
			int end = locs.size() - 1;
			line(Field, cv::Point(0, 1500 / 10), cv::Point(FIELD_WIDTH / 10, 1500 / 10), cv::Scalar(255, 255, 255));
			line(Field, cv::Point(0, 4440 / 10), cv::Point(FIELD_WIDTH / 10, 4440 / 10), cv::Scalar(255, 255, 255));
			cv::Point Point2 = cv::Point((locs.at(end).getX() + FIELD_WIDTH / 2) / 10, (locs.at(end).getZ()) / 10);
			circle(Field, Point2, 3, cv::Scalar(0, 255, 0), 3);
		}
		cameraLoc.setAngles(phi, theta, 0);
		cv::imshow("Source Video", videoFrame);
		imshow("Field", Field);
		//Delays to show image
		cv::waitKey(10);

	}

	return EXIT_SUCCESS;
}