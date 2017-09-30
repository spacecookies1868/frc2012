#ifndef AUTONOMOUSCONTROLLER_H_
#define AUTONOMOUSCONTROLLER_H_

#include "ControlBoard.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "ShooterController.h"
#include "CameraController.h"
#include "PopUpController.h"
#include "RobotModel.h"
#include <vector>
#include <string>

class AutoCommand {
public:
    virtual void Start() = 0;
    virtual bool IsDone() = 0;
};

enum AutoMode {
	kTestMode, kJustShoot, kDriveForwardAutonomous, kJustLowerBridge, kShootThenLowerBridge, kSpitOutBalls
};

class DriveCommand : public AutoCommand {
public:
	DriveCommand(double myDistance, double mySpeed, DriveController *myDriveController, string myMsg = "")
		: distance(myDistance), 
		  speed(mySpeed),
		  driveController(myDriveController) {};
	virtual void Start() { 
		driveController->RequestAutoDrive(distance, speed); 
		printf("just requested auto drive from auto controller\n");
	};
	virtual bool IsDone() { return driveController->AutoDriveDone(); };
private:
	double distance;
	double speed;
	DriveController *driveController;
	
};

class PivotCommand : public AutoCommand {
public:
	PivotCommand(double myAngle, double mySpeed, DriveController *myDriveController)
		: angle(myAngle), 
		  speed(mySpeed),
		  driveController(myDriveController){};
	virtual void Start() { driveController->RequestAutoPivot(angle, speed); };
	virtual bool IsDone() { return driveController->AutoPivotDone(); };
private:
	double angle;
	double speed;
	DriveController *driveController;
	
};

class WaitingCommand : public AutoCommand {
public:
	WaitingCommand(double myWaitTimeSec) : waitTimeSec(myWaitTimeSec) {
		timer = new Timer();
	};
	virtual void Start() { timer->Start(); };
	virtual bool IsDone() { return (timer->Get() >= waitTimeSec); };
		
private:
		double waitTimeSec;
		Timer *timer;
};

class ChangeConveyorStateCommand : public AutoCommand {
public:
	ChangeConveyorStateCommand(bool isOn, ShooterController *myShooterController) : on(isOn), shooterController(myShooterController) {} ;
	virtual void Start() {
		if (on) shooterController->RequestInfiniteConveyor();
		else shooterController->RequestInfiniteConveyorOff();
	}
	virtual bool IsDone() { return true; }
	
private:
	bool on;
	ShooterController *shooterController;
};

class FlywheelOnCommand : public AutoCommand {
public:
	FlywheelOnCommand(bool setOn, ShooterController *myShooterController) : on(setOn), shooterController(myShooterController) {};
	virtual void Start() { shooterController->RequestFlywheel(on); };
	virtual bool IsDone() { return true; }

private:
	bool on;
	ShooterController *shooterController;
};

class ShootCommand : public AutoCommand {
public:
	ShootCommand(ShooterController *myShooterController) : shooterController(myShooterController) {};
	virtual void Start() { /* implementation here */};
	virtual bool IsDone() { /* implementation here */ return true;};
private:
	ShooterController *shooterController;
};

class HoodSetCommand : public AutoCommand {
public:
	HoodSetCommand(bool setExtended, RobotModel *myRobot) : extended(setExtended), robot(myRobot) {};
	virtual void Start() {
		if (extended) robot->RaiseHood();
		else robot->LowerHood();
	};
	virtual bool IsDone() { return true; };
private:
	bool extended;
	RobotModel *robot;
};

class FlywheelSpeedSetCommand : public AutoCommand {
public:
	FlywheelSpeedSetCommand(int atPos, ShooterController *myShooterController) : pos(atPos), shooterController(myShooterController) {};
	virtual void Start() { shooterController->SetShooterSpeedByPosition(pos); };
	virtual bool IsDone() { return true; };
private:
	int pos;
	ShooterController *shooterController;
};

class SetIntakeCommand : public AutoCommand {
public:
	SetIntakeCommand(bool setDown, RobotModel *myRobot) : down(setDown), robot(myRobot) {};
	virtual void Start() {
		if (down) robot->LowerIntake();
		else robot->RaiseIntake();
	}
	virtual bool IsDone() { return true; }
private:
	bool down;
	RobotModel *robot;
};

class SetRampLegsCommand : public AutoCommand {
public:
	SetRampLegsCommand(bool setDown, RobotModel *myRobot) : down(setDown), robot(myRobot) {};
	virtual void Start() {
		if (down) robot->LowerRampLegs();
		else robot->RaiseRampLegs();
	}
	virtual bool IsDone() { return true; }
private:
	bool down;
	RobotModel *robot;
};

class SetRollerSpeedCommand : public AutoCommand {
public:
	SetRollerSpeedCommand(double mySpeed, RobotModel *myRobot) : speed(mySpeed), robot(myRobot) {};
	virtual void Start() {
		robot->SetRollerSpeed(speed);
	}
	virtual bool IsDone() { return true; }
private:
	double speed;
	RobotModel *robot;
	
};

class SetConveyorSpeedCommand : public AutoCommand {
public:
	SetConveyorSpeedCommand(double mySpeed, RobotModel *myRobot) : speed(mySpeed), robot(myRobot) {};
	virtual void Start() {
		robot->SetConveyorSpeed(speed);
	}
	virtual bool IsDone() { return true; }
private:
	double speed;
	RobotModel *robot;
	
};

class AutonomousController
{
public:
	AutonomousController(RobotModel *myRobot,
			RemoteController *myControlBoard,
			CameraController *myCameraController,
			DriveController *myDriveController,
			ShooterController *myShooterController,
			PopUpController *myPopUpController);
	void CreateQueue();
	void StartAutonomous();
	virtual ~AutonomousController();
	void Update(double currTimeSec, double deltaTimeSec);

private:
	vector<AutoCommand*> commandSequence;
	RobotModel *robot;
	RemoteController *controlBoard;
	CameraController *cameraController;
	DriveController *driveController;
	ShooterController *shooterController;
	PopUpController *popUpController;
	unsigned int autoMode;
	unsigned int sequenceNumber;
	double initialWait;
	bool doneWithSequence;
	double minFlywheelSpinUpTime;
};

#endif /*AUTONOMOUSCONTROLLER_H_*/
