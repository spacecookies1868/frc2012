#include "CameraController.h"
#include "WPILib.h"
#include "Debugging.h"
#include "RectangleTarget.h"
#include <math.h>

static RectangleDescriptor rectangleDescriptor = {
		10, 	// minWidth
		200, 	// maxWidth
		10, 		// minHeight
		100		// maxHeight

		//may need to change min sizes

};

static CurveOptions curveOptions = {	IMAQ_UNIFORM_REGIONS,	// extractionMode
		75, 				// threshold
		IMAQ_NORMAL, 		// filterSize
		25, 				// minLength
		15, 				// rowStepSize 
		15, 				// columnStepSize
		10, 				// maxEndPointGap
		1,					// onlyClosed
		0					// subpixelAccuracy
};

static ShapeDetectionOptions shapeOptions = {
		IMAQ_GEOMETRIC_MATCH_ROTATION_INVARIANT,	// mode
		NULL,			// angle ranges
		0,				// num angle ranges
		{75, 125},		// scale range
		500				// minMatchScore
};

CameraController::CameraController(RobotModel *myRobot, RemoteController *myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;
	cameraImageRequested = false;
	
	RefreshIni();
	
	hascamera = robot->HasCamera();

}
#ifdef USE_CAMERA

void CameraController::Update(double currTimeSec, double deltaTimeSec) {
	if(hascamera == false) {
		return ;
	}
	if (humanControl->CameraImageDesired() || cameraImageRequested) {
		//printf("Target finding disabled until we resolve memory management issues.\n"
		//		"Edit CameraController::Update() to enable.\n");
		ENTRY("CameraImageDesired");

		HSLImage *cameraImage = robot->GetCameraImage();

		latestTargets = FindTargets(cameraImage);

		delete cameraImage;

		cameraImageRequested = false;

		//		printf("Processed image... found %d targets.\n", latestTargets.size());
		//		for (unsigned int i=0; i < latestTargets.size(); i++) {
		//			printf("Rect %d : xCenter = %f, yCenter = %f, w = %f, h = %f, score = %f \n", 
		//					i+1, latestTargets[i].xCenter, latestTargets[i].yCenter, 
		//					latestTargets[i].width, latestTargets[i].height, 
		//					latestTargets[i].score);
		//		}
		// robot->dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "%d targets Angle %f", 
		//		latestTargets.size(), GetAngleToTopTarget());
		printf("Found %d targets. Angle error %f\n", latestTargets.size(), GetAngleToTopTarget());

		robot->dsLCD->UpdateLCD();
	}
}

vector<RectangleTarget> CameraController::FindTargets(HSLImage *image) {

	vector<RectangleTarget> targets;
	if (image != NULL) {
		//MonoImage  *luminancePlane = image->GetLuminancePlane();	

		//Image* imaqImage = luminancePlane->GetImaqImage();
		int numMatches;
		int rc;
		Image *thresholdImage = imaqCreateImage(IMAQ_IMAGE_U8, DEFAULT_BORDER_SIZE);
		Image *convexHullImage = imaqCreateImage(IMAQ_IMAGE_U8, DEFAULT_BORDER_SIZE);
		Image *equalizedImage = imaqCreateImage(IMAQ_IMAGE_U8, DEFAULT_BORDER_SIZE);

		Range thresholdRangeR;
		thresholdRangeR.minValue = colorThresholdRMin;
		thresholdRangeR.maxValue = colorThresholdRMax;
		Range thresholdRangeG;
		thresholdRangeG.minValue = colorThresholdGMin;
		thresholdRangeG.maxValue = colorThresholdGMax;
		Range thresholdRangeB;
		thresholdRangeB.minValue = colorThresholdBMin;
		thresholdRangeB.maxValue = colorThresholdBMax;

		// rc = imaqWriteBMPFile(image->GetImaqImage(), "rawImage.bmp", FALSE, NULL);
		// if (rc == 0) printf("Raw image write error # %d", imaqGetLastError());

		rc = imaqColorThreshold(thresholdImage, image->GetImaqImage(), 255, IMAQ_RGB, &thresholdRangeR, &thresholdRangeG, &thresholdRangeB);
		if (rc == 0) printf("Thresholding error # %d", imaqGetLastError());

		// rc = imaqWriteBMPFile(thresholdImage, "thresholdImage.bmp", FALSE, NULL); 
		// if (rc == 0) printf("Threshold image write error # %d", imaqGetLastError());

		rc = imaqConvexHull(convexHullImage, thresholdImage, 1);
		if (rc == 0) printf("Convex hull error # %d", imaqGetLastError());

		// rc = imaqWriteBMPFile(convexHullImage, "convexHullImage.bmp", FALSE, NULL); 
		// if (rc == 0) printf("Convex hull image write error # %d", imaqGetLastError());

		rc = imaqDispose(thresholdImage);
		if (rc == 0) printf("Delete threshold image error # %d", imaqGetLastError());

		rc = imaqEqualize(equalizedImage, convexHullImage, 0, 255, NULL);
		if (rc == 0) printf("Equalized image error # %d", imaqGetLastError());

		rc = imaqWriteBMPFile(equalizedImage, "equalizedImage.bmp", FALSE, NULL); 
		if (rc == 0) printf("Equalized image write error # %d", imaqGetLastError());

		rc = imaqDispose(convexHullImage);
		if (rc == 0) printf("Delete convex hull image error # %d", imaqGetLastError());

		RectangleMatch* rectangles = imaqDetectRectangles(equalizedImage, &rectangleDescriptor, &curveOptions, &shapeOptions, NULL, &numMatches);
		rc = imaqDispose(equalizedImage);
		if (rc == 0) printf("Delete equalized image error # %d", imaqGetLastError());

		vector<RectangleMatch> results;
		if (rectangles == NULL) {
			return targets;
		}
		else {
			for (int i=0; i<numMatches; i++) {
				results.push_back(rectangles[i]);
			}
		}
		imaqDispose(rectangles);

		for (unsigned int i=0; i<results.size(); i++) {
			RectangleTarget target;
			RectangleMatch r = results.at(i);
			/***
		printf("R %d C0 : %f %f C1 %f %f C2 %f %f C3 %f %f \n", i, 
				r.corner[0].x, r.corner[0].y, 
				r.corner[1].x, r.corner[1].y,
				r.corner[2].x, r.corner[2].y,
				r.corner[3].x, r.corner[3].y);
			 ***/
			target.score = r.score;
			target.xPos = r.corner[0].x;
			target.yPos = r.corner[0].y;
			target.xCenter = (r.corner[0].x)+(r.width/2);
			target.yCenter = (r.corner[0].y)+(r.height/2);
			target.width = r.width;
			target.height = r.height;
			targets.push_back(target);
		}
	}
	return targets;
}
void CameraController::RequestNewCameraImage(){
	cameraImageRequested = true;

}
vector<RectangleTarget> CameraController::GetTargets() {
	return latestTargets;
}

double CameraController::GetAngleToTopTarget(){
	double TopTargetCenter = GetTopTargetCenter();
	//If the TopTargetCenter didn't succeed, then don't return a value to pivot
	if (TopTargetCenter < 0) return 0.0;
	double aimAngleError;
	double distToOurCenter = PixelsToInches(320/tan(DegreesToRadians(23.5)));
	double aimDistanceError = PixelsToInches(320 - TopTargetCenter );
	printf("Range: %f Dist Error: %f inches TTC: %f\n", distToOurCenter, aimDistanceError, TopTargetCenter);
	aimAngleError = atan(aimDistanceError/distToOurCenter)* (180/PI);
	//the gyro faces out the front of the robot, but the camera faces out the back of the robot
	return -aimAngleError;
}

void CameraController::RefreshIni() {
	kAngle = (robot->pini)->getf("CameraControllerValues", "ANGLE_OFFSET_CONSTANT", 0.6);
	alignDifferenceThreshold = (robot->pini)->getf("CameraControllerValues", "ALIGN_RECT_DIFFERENCE_THRESHOLD", 10.0);
	topTargetThreshold = (robot->pini) ->getf("CameraControllerValues", "TOP_TARGET_THRESHOLD", 160.0);
	colorThresholdRMin = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_R_MIN", 0);
	colorThresholdRMax = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_R_MAX", 78);
	colorThresholdGMin = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_G_MIN", 0);
	colorThresholdGMax = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_G_MAX", 130);
	colorThresholdBMin = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_B_MIN", 50);
	colorThresholdBMax = (robot->pini)->geti("CameraControllerValues", "COLOR_THRESHOLD_B_MAX", 255);
}

double CameraController::GetTopTargetCenter() {
	int top = 0;
	printf("Processed image... found %d targets.\n", latestTargets.size());
	for (unsigned int i=0; i < latestTargets.size(); i++) {
		printf("Rect %d : xCenter = %f, yCenter = %f, w = %f, h = %f, score = %f \n", 
				i+1, latestTargets[i].xCenter, latestTargets[i].yCenter, 
				latestTargets[i].width, latestTargets[i].height, 
				latestTargets[i].score);
	}

	if (latestTargets.size() == 4){
		//if the script finds 4 rectangles, the one with the smallest y-center is the top target.
		for (unsigned int i = 0; i < latestTargets.size(); i++) {
			if (latestTargets[top].yCenter > latestTargets[i].yCenter)
				top = i;
		}
		return latestTargets[top].xCenter;
	}
	else if (latestTargets.size() == 3){
		/** If the script finds 3 rectangles, two are going to be aligned on either the X or Y axis. 
		 *  If two are aligned vertically, the target with the smaller y-center is going to be the top.
		 *  If two are aligned horizontally, the remaining target is either the top/bottom target, 
		 *  which have approximately the same xCenter.
		 */

		if (AlignedVertical(latestTargets[0], latestTargets[1])) {
			printf("AV 0 1, top and bottom targets\n");
			return (HigherTarget(0, 1).xCenter);
		}
		else if (AlignedVertical(latestTargets[1], latestTargets[2])) {
			printf("AV 1 2, top and bottom targets\n");
			return (HigherTarget(1, 2).xCenter);
		}
		else if (AlignedVertical(latestTargets[0], latestTargets[2])) {
			printf("AV 0 2, top and bottom targets\n");
			return (HigherTarget(0, 2).xCenter);
		}
		else if (AlignedHorizontal(latestTargets[0], latestTargets[1])) {
			printf("AH 0 1, two middle targets\n");
			return latestTargets[2].xCenter;
		}
		else if (AlignedHorizontal(latestTargets[1], latestTargets[2])) {
			printf("AH 1 2, two middle targets\n");
			return latestTargets[0].xCenter;
		}
		else if (AlignedHorizontal(latestTargets[0], latestTargets[2])) {
			printf("AH 0 2, two middle targets\n");
			return latestTargets[1].xCenter;
		}
	}
	else if (latestTargets.size() == 2){
		RectangleTarget higher = HigherTarget(0,1);
		RectangleTarget lower = LowerTarget(0, 1);
		//first two cases are top/bottom and middle targets
		if (AlignedVertical(latestTargets[0], latestTargets[1])){
			printf("AV 0 1, top and bottom targets\n");
			return higher.xCenter;
		}
		else if (AlignedHorizontal(latestTargets[0], latestTargets[1])) {
			//average of the two middle targets' centers is the center of the top target
			printf("AH 0 1, two middle targets\n");
			return (latestTargets[0].xCenter + latestTargets[1].xCenter)/2;
		} 
		else {
			printf("Targets found are both on different axes\n");
			//arbitrary frame threshold for top target center
			if (higher.yCenter <= topTargetThreshold) {
				printf("Assumed the higher target is the top target\n");
				return higher.xCenter;
			}
			else{ 
				printf("Assumed the lower target is the bottom target\n");
				return lower.xCenter;}
		}
	}
	else if (latestTargets.size() == 1) {
		return latestTargets[0].xCenter;
	}
	//in any other case, there is no guarantee of locating the top target
	return -1.0;

}
//all mini-functions to help out with image analysis algorithm
double CameraController::DegreesToRadians(double degrees){
	double radians = degrees * (PI/180);
	return radians;
}

bool CameraController::AlignedVertical(RectangleTarget rect1, RectangleTarget rect2) {
	bool sameAxis = true;
	if ((fabs(rect1.xCenter - rect2.xCenter)) <= alignDifferenceThreshold)
		return sameAxis;
	else return !sameAxis;
}

bool CameraController::AlignedHorizontal(RectangleTarget rect1, RectangleTarget rect2){
	bool sameAxis = true;
	if ((fabs(rect1.yCenter - rect2.yCenter)) <= alignDifferenceThreshold)
		return sameAxis;
	else return !sameAxis;
}

RectangleTarget CameraController::HigherTarget(int a, int b){
	if (latestTargets[a].yCenter > latestTargets[b].yCenter) return latestTargets[a];
	else return latestTargets[b];

}

RectangleTarget CameraController::LowerTarget(int a, int b){
	if (latestTargets[a].yCenter < latestTargets[b].yCenter) return latestTargets[a];
	else return latestTargets[b];
}

double CameraController::PixelsToInches(double pixels){
	//using the height of the top target
	double pixelsPerInch = latestTargets[0].height/18.0;
	return pixels/pixelsPerInch;
}

#endif

CameraController::~CameraController()
{
}
