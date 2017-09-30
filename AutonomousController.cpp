#include "AutonomousController.h"

AutonomousController::AutonomousController(RobotModel *myRobot,
		RemoteController *myControlBoard,
		CameraController *myCameraController,
		DriveController *myDriveController,
		ShooterController *myShooterController,
		PopUpController *myPopUpController) : 
		robot(myRobot),
		controlBoard(myControlBoard),
		driveController(myDriveController),
		shooterController(myShooterController),
		popUpController(myPopUpController)

{ }

void AutonomousController::CreateQueue() {
	int bridge = (robot->pini)->geti("AUTONOMOUS", "AutoMode", 1); // 1 is with bridge - default
	int useSwitch = 0; //robot->pini->geti("AUTONOMOUS", "UseSwitch", 1); // 1 means to listen to the switch
	bool spitOutBalls = robot->pini->geti("AUTONOMOUS", "SpitOutBalls", 0); // 1 means spit out balls, 0 means normal auto (shooting)
	
	double firstConveyorDuration = robot->pini->getf("AUTONOMOUS", "FirstConveyorDuration", 0.51);
	double pauseBetween = robot->pini->getf("AUTONOMOUS", "PauseBetween", 2.01);
	double secondConveyorDuration = robot->pini->getf("AUTONOMOUS", "SecondConveyorDuration", 1.51);
	
	printf("1: %f pause: %f 2: %f", firstConveyorDuration, pauseBetween, secondConveyorDuration);
	
	unsigned int shootingDistance;
	int shootFarKeyDesired = (robot->pini)->geti("AUTONOMOUS", "ShootFar", 1); // 1 is far key, 0 is near key
	//bool shootFarKeyDesired = controlBoard->ShootFarKeyDesired();
	if (shootFarKeyDesired) shootingDistance = kKeyFar;
	else shootingDistance = kKeyNear;
	
	bool tipBridgeDesired = controlBoard->AutoTipBridgeDesired();
	
	float drive_dist;
	if (shootFarKeyDesired){
		drive_dist = (robot->pini)->getf("AUTONOMOUS", "ForwardDist", -5.71);
	} else {
		// we're shooting from the near position, so we need to drive farther
		drive_dist = robot->pini->getf("AUTONOMOUS", "NearKeyForwardDist", -6.71);
	}
	
	printf("Tipping bridge? %s\n", tipBridgeDesired ? "YES" : "NO");
	printf("Starting from %s key.\n", shootFarKeyDesired ? "FAR" : "NEAR");
	printf("If tipping bridge, drive distance is %f\n", drive_dist);
	
	/*if (useSwitch) {
		if (tipBridgeDesired) bridge = 1;
		else bridge = 0;
		printf("autonomous using switch\n");
	} else {
		printf("autonomous not using switch\n");
	}*/
	
	if (spitOutBalls) {
		printf("Autonomous Mode - Spit out balls\n");
		autoMode = kSpitOutBalls;
	} else if (tipBridgeDesired) {
		printf("Autonomous Mode - Bridge - with distance: %f\n", drive_dist);
		autoMode = kShootThenLowerBridge;
	} else {
		printf("Autonomous Mode - Shooting only\n");
		autoMode = kJustShoot;
	}

	sequenceNumber = 0;
	doneWithSequence = false;
	//bool lowerBridge = false; // set to true if you want to try lowering the bridge
	minFlywheelSpinUpTime = 0.8; // not used anymore
	initialWait = (robot->pini)->getf("AUTONOMOUS", "initialWaitTime", 1.01);
	//if (initialWait < minFlywheelSpinUpTime) initialWait = minFlywheelSpinUpTime;

	switch (autoMode) {

	case (kTestMode):
		commandSequence.push_back(new DriveCommand(4.0, 0.8, driveController));
		commandSequence.push_back(new WaitingCommand(2.0));
		commandSequence.push_back(new PivotCommand(45, 0.8, driveController));
	break;

	default:
	case (kJustShoot):
		commandSequence.push_back(new HoodSetCommand(true, robot));
		commandSequence.push_back(new FlywheelSpeedSetCommand(shootingDistance, shooterController));
		commandSequence.push_back(new FlywheelOnCommand(true, shooterController));
		commandSequence.push_back(new WaitingCommand(initialWait));
		commandSequence.push_back(new ChangeConveyorStateCommand(true, shooterController));
		commandSequence.push_back(new WaitingCommand(firstConveyorDuration)); // move first ball
		commandSequence.push_back(new ChangeConveyorStateCommand(false, shooterController));
		commandSequence.push_back(new WaitingCommand(pauseBetween)); // wait for flywheel
		commandSequence.push_back(new ChangeConveyorStateCommand(true, shooterController));
		commandSequence.push_back(new WaitingCommand(secondConveyorDuration));  // move second ball
		commandSequence.push_back(new ChangeConveyorStateCommand(false, shooterController));
		commandSequence.push_back(new FlywheelOnCommand(false, shooterController));
	break;

	case (kDriveForwardAutonomous):
		commandSequence.push_back(new DriveCommand(7.0, 0.8, driveController));
	break;

	case (kJustLowerBridge):
		commandSequence.push_back(new DriveCommand(-5.0, 0.8, driveController));
	
		// lowering ramp manipulator
		commandSequence.push_back(new SetRampLegsCommand(true, robot));
		commandSequence.push_back(new WaitingCommand(driveController->GetIntakeLoweringDelay()));
		commandSequence.push_back(new SetIntakeCommand(true, robot));
		
	break;

	case (kShootThenLowerBridge):
		// shooting
		commandSequence.push_back(new HoodSetCommand(true, robot));
		commandSequence.push_back(new FlywheelSpeedSetCommand(kKeyNear, shooterController));
		commandSequence.push_back(new FlywheelOnCommand(true, shooterController));
		commandSequence.push_back(new WaitingCommand(initialWait));
		commandSequence.push_back(new ChangeConveyorStateCommand(true, shooterController));
		commandSequence.push_back(new WaitingCommand(firstConveyorDuration)); // move first ball
		commandSequence.push_back(new ChangeConveyorStateCommand(false, shooterController));
		commandSequence.push_back(new WaitingCommand(pauseBetween)); // wait for flywheel
		commandSequence.push_back(new ChangeConveyorStateCommand(true, shooterController));
		commandSequence.push_back(new WaitingCommand(secondConveyorDuration));  // move second ball
		commandSequence.push_back(new ChangeConveyorStateCommand(false, shooterController));
		commandSequence.push_back(new FlywheelOnCommand(false, shooterController));	// turn flywheel off


		// driving to bridge
		commandSequence.push_back(new DriveCommand(drive_dist, 0.6, driveController));
		commandSequence.push_back(new WaitingCommand(0.5));
		
		// lowering manipulator
		commandSequence.push_back(new SetRampLegsCommand(true, robot));
		commandSequence.push_back(new WaitingCommand(driveController->GetIntakeLoweringDelay()));
		commandSequence.push_back(new SetIntakeCommand(true, robot));
			
		// driving away
		commandSequence.push_back(new WaitingCommand(1.0));
		commandSequence.push_back(new DriveCommand(1.0, 0.7, driveController));
		commandSequence.push_back(new WaitingCommand(1.0));
		commandSequence.push_back(new DriveCommand(1.0, 0.7, driveController));
		
		break;
		
	case (kSpitOutBalls):
			
		// Wait at beginning
		commandSequence.push_back(new WaitingCommand(3.8));
		
		// Start reverse rollers and reverse conveyor
		commandSequence.push_back(new SetRollerSpeedCommand(-0.9, robot));
		commandSequence.push_back(new SetConveyorSpeedCommand(-0.9, robot));

		// Wait ~10sec
		commandSequence.push_back(new WaitingCommand(4.0));
		
		// Stop rollers and conveyor
		commandSequence.push_back(new SetRollerSpeedCommand(0, robot));
		commandSequence.push_back(new SetConveyorSpeedCommand(0, robot));
		
		break;
	}
}

void AutonomousController::StartAutonomous() {
	CreateQueue();
	sequenceNumber = 0;
	doneWithSequence = false;
	if (commandSequence.size() > 0) {
		commandSequence.at(sequenceNumber)->Start();
		printf("Just started command %d\n", sequenceNumber);
	}
	else {
		doneWithSequence = true;
	}
}

void AutonomousController::Update(double currTimeSec, double deltaTimeSec) {
	if (!doneWithSequence) {
		if (commandSequence.at(sequenceNumber)->IsDone()) {
			sequenceNumber++;
			if (sequenceNumber >= commandSequence.size()) {
				doneWithSequence = true;
				printf("Done with autonomous.\n");
			} else {
				commandSequence.at(sequenceNumber)->Start();
				printf("Just started command %d at %f\n", sequenceNumber, currTimeSec);
			}
		}
	}
}



AutonomousController::~AutonomousController()
{
}
