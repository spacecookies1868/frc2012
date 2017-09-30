#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "DriveController.h"
#include "ShooterController.h"
#include "CameraController.h"
#include "PopUpController.h"
#include "DashboardDataSender.h"
#include "AutonomousController.h"
#include "Debugging.h"

class MainProgram : public IterativeRobot
{
		
public:
	
	RemoteController *humanControl;
	RobotModel *robot;
	DriveController *driveController;
	ShooterController *shooterController;
	CameraController *cameraController;
	PopUpController *popUpController;
	AutonomousController *autoController;
	ADXL345_I2C *accel;
	
	bool spitOutBalls;
	
	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;
	
	DashboardDataSender *dashboardDataSender;
	Ini *ini;
	
	MainProgram(void) {
		// constructor
		robot = new RobotModel();
		ini = robot->pini;
		humanControl = new ControlBoard(robot);
		cameraController = new CameraController(robot, humanControl);
		driveController = new DriveController(robot, humanControl,cameraController);
		shooterController = new ShooterController(robot, humanControl);
		popUpController = new PopUpController(robot, humanControl);
		autoController = new AutonomousController(robot, humanControl, cameraController, driveController, shooterController, popUpController);
		
		
		accel = new ADXL345_I2C(1/*,DataFormat_Range(kRange_2G*/);
		dashboardDataSender = new DashboardDataSender();
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		
		SetPeriod(0.01);
	}
	
	void RobotInit(void) {
		robot->EnableCompressor();
		// Actions that are performed once (and only once) when the robot is initialized
	}
	
	void DisabledInit(void) {
		// Actions that are performed once as soon as the robot becomes disabled
		driveController->Reset();
		//
		delete robot->pini ; // delete the ini file class ; 
		// Now reinitialize it so it will read it back in.
		robot->pini = new Ini("/robot.ini");

	}

	void AutonomousInit(void) {
		// Actions that are performed once as soon as the autonomous period begins
		robot->ResetGyro();
		shooterController->RefreshIni();
		cameraController->RefreshIni();
		driveController->RefreshIni();
		driveController->Reset();
		driveController->Update(0,0);
		humanControl->ReadControls();
		autoController->StartAutonomous();
		spitOutBalls = robot->pini->geti("AUTONOMOUS", "SpitOutBalls", 0);
	}

	void TeleopInit(void) {
		// Actions that are performed once as soon as the teleop period begins
		printf("NEW NEW joysticks....\n");
		double xAxis = accel->GetAcceleration(ADXL345_I2C::kAxis_X);
			printf("x value = %f\n", xAxis);
			
		double yAxis = accel->GetAcceleration(ADXL345_I2C::kAxis_Y);
			printf("y value = %f\n", yAxis);
		double zAxis = accel->GetAcceleration(ADXL345_I2C::kAxis_Z);
			printf("z value = %f\n", zAxis);
		
		driveController->Reset();
		shooterController->Reset();
		shooterController->RefreshIni();
		cameraController->RefreshIni();
		driveController->RefreshIni();

	}
	
	void DisabledPeriodic(void)  {
		// Actions that are performed periodically while the robot is disabled
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "FW dial %f", (humanControl->GetFlywheelSpeedAdjustMultiplier() - 1));
		robot->dsLCD->UpdateLCD();
	}

	void AutonomousPeriodic(void) {
		// Actions that are performed periodically during autonomous
		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;

		humanControl->ReadControls();

		autoController->Update(currTimeSec, deltaTimeSec);
		
#ifdef USE_CAMERA
		cameraController->Update(currTimeSec, deltaTimeSec); // MUST update before driveController so that driveController
#endif
		// has new images to work with.
		driveController->Update(currTimeSec, deltaTimeSec);
		
		if (!spitOutBalls) {
			shooterController->Update(currTimeSec, deltaTimeSec);
		}
		//testing the beam sensor
		//DO_PERIODIC(500, printf("The status of the beam sensor is %d\n", robot->popUpBeamSensor->Get()));
		popUpController->Update(currTimeSec, deltaTimeSec);
	}

	void TeleopPeriodic(void) {
		// Actions that are performed periodically during teleop
		
		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;
		
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "FW dial %f        ", (humanControl->GetFlywheelSpeedAdjustMultiplier() - 1));
		robot->PrintDebugInfoToLCD();
		robot->dsLCD->UpdateLCD();
		
		humanControl->ReadControls();
#ifdef USE_CAMERA
		cameraController->Update(currTimeSec, deltaTimeSec); // MUST update before driveController so that driveController
															 // has new images to work with.
#endif
		driveController->Update(currTimeSec, deltaTimeSec);
		shooterController->Update(currTimeSec, deltaTimeSec);
		//testing the beam sensor
		//DO_PERIODIC(500, printf("The status of the beam sensor is %d\n", robot->popUpBeamSensor->Get()));
		popUpController->Update(currTimeSec, deltaTimeSec);
		
		robot->dsLCD->UpdateLCD();

		//dashboardDataSender->sendVisionData(0.0, 0.0, 0.0, 0.0, cameraController->GetTargets());
		dashboardDataSender->sendIOPortData();

	}



	void DisabledContinuous(void) {
		// Actions that are performed repeatedly as fast as possible while disabled
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "FW dial %f", (humanControl->GetFlywheelSpeedAdjustMultiplier() - 1));
				robot->dsLCD->UpdateLCD();
	}

	void AutonomousContinuous(void)	{
		// Actions that are performed repeatedly as fast as possible during autonomous
	}

	void TeleopContinuous(void) {
		// Actions that are performed repeatedly as fast as possible during teleop
	}
			
};

START_ROBOT_CLASS(MainProgram);
