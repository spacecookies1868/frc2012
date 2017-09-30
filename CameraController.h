#ifndef CAMERACONTROLLER_H_
#define CAMERACONTROLLER_H_

#include <vector>
#include "WPILib.h"
#include "RobotModel.h"
#include "RemoteControl.h"
#include "RectangleTarget.h"
#include "Debugging.h"

#define PI 3.1415927

class CameraController
{
public:
	CameraController(RobotModel *myRobot, RemoteController *myHumanControl);
	
#ifdef USE_CAMERA
	
	vector<RectangleTarget> FindTargets(HSLImage *image);
	
	void Update(double currTimeSec, double deltaTimeSec);
	
	vector<RectangleTarget> GetTargets();
	
	double GetTopTargetCenter();
	
	double PixelsToInches(double pixels);
	
	double DegreesToRadians(double degrees);
	
	double GetAngleToTopTarget();
	
	void RequestNewCameraImage();

	bool AlignedVertical(RectangleTarget rect1, RectangleTarget rect2);
	
	bool AlignedHorizontal(RectangleTarget rect1, RectangleTarget rect2);
	
	RectangleTarget HigherTarget(int a, int b);
	RectangleTarget LowerTarget(int a, int b);
	
#endif
	void RefreshIni();
	virtual ~CameraController();
	
private:
	RobotModel *robot;
	RemoteController *humanControl;
	
	vector<RectangleTarget> latestTargets;
	double kAngle;
	double alignDifferenceThreshold;
	double topTargetThreshold;
	int colorThresholdRMin;
	int colorThresholdRMax;
	int colorThresholdGMin;
	int colorThresholdGMax;
	int colorThresholdBMin;
	int colorThresholdBMax;
	bool cameraImageRequested;
	bool hascamera ;
	
};

#endif /*CAMERACONTROLLER_H_*/
