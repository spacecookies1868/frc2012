#include "DriveController.h"
//#include "RobotModel.h"
#include "Debugging.h"
#include <math.h>

//#define FIX_SWITCHES 1

DriveController::DriveController(RobotModel *myRobot, RemoteController *myHumanControl, CameraController *myCameraController){
	robot = myRobot;
	humanControl = myHumanControl;
	cameraController = myCameraController;
	m_stateVal = kTeleopDrive; // Automatically goes to teleop and stays there
	autoDriveRequested = false;
	autoDriveDone = true;
	autoInitialGyroAngle = 0.0;
	currAngle = 0.0;
	originallyHighGear = true;
	
	intakeLoweringDelay = 0.0 ;
	
	intakeLoweringDelayStart = 0.0;
	
	autoPivotRequested = false;
	autoPivotDone = true;
	autoPivotGoal = 0.0;
	autoPivotSpeed = 0.0;
	adjustedAutoPivotSpeed = 0.0;
	autoDiffPivotError = 0;
	
	pidExitCountdownStartTime = 0.0;
	pidExitDelay = 1.0;
	
	mustReset = false;
	gyroVal = 0.0;

	RefreshIni();

}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {

	gyroVal = robot->GetGyroAngle();
	
	DO_PERIODIC(50, robot->dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Gyro %f      ", gyroVal));

	if (mustReset) {
		m_stateVal = kReset;
		mustReset = false;
	}
	/**
	 * Drive State Machine
	 */
	switch (m_stateVal)
	{
	case (kInitialize):
					break;

	case (kReset):
	autoDriveRequested = false;
	autoDriveDone = true;
	autoPivotRequested = false;
	autoPivotDone = true;
	robot->SetBrakeOff();
	if (humanControl->LowGearDesired()) {
		robot->ShiftToLowGear();
	} else {
		robot->ShiftToHighGear();
	}
	
	if (humanControl->BrakeDesiredOn()) {
		robot->SetBrakeOn();
	} else {
		robot->SetBrakeOff();
	}
	
	/*if (humanControl->RampManipDownDesired()) {
		robot->LowerRampManipulator();
	} else {
		robot->RaiseRampManipulator();
	}*/
	//robot->RaiseRampManipulator();
	nextState = kTeleopDrive;
	break;

	case (kTeleopDrive):

		//Setting left wheel speed
		robot->SetWheelSpeed(humanControl->GetLeftWheelDesiredSpeed(), RobotModel::kLeftWheel);
	
		//Setting right wheel speed
		robot->SetWheelSpeed(humanControl->GetRightWheelDesiredSpeed(), RobotModel::kRightWheel);
	
		
		//DO_PERIODIC(25, printf("Raw encoder R: %f L: %f\n", robot->GetWheelEncoderValue(RobotModel::kRightWheel),
			//					robot->GetWheelEncoderValue(RobotModel::kLeftWheel)));
		//Shifting gears
		
#ifdef FIX_SWITCHES
	
		
		if (humanControl->LowGearDesired()) {
			robot->ShiftToLowGear();
			// robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Low gear" );
			// robot->dsLCD->UpdateLCD();
		} else {
			robot->ShiftToHighGear();
			// robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "High gear" );
			// robot->dsLCD->UpdateLCD();
		}
		
		if (humanControl->RampManipDownDesired()) {
			robot->LowerRampLegs();
			intakeLoweringDelayStart = currTimeSec;
			printf("beginning delay\n");
			//robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Ramp manip lowered" );
			//robot->dsLCD->UpdateLCD();
		} else {
			robot->RaiseRampLegs();
			robot->RaiseIntake();
			//robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Ramp manip raised" );
			//robot->dsLCD->UpdateLCD();
		}
		//Delay between command to lowering ramp legs and lower intake so that legs are fully extended before 
		if (intakeLoweringDelayStart != 0 && currTimeSec > (intakeLoweringDelayStart + intakeLoweringDelay)) {
			printf("delay finished\n");
			robot->LowerIntake();
			intakeLoweringDelayStart = 0.0;
		}


#else
		
			// THIS IS THE ACTIVE CODE PATH - Hansa - 4/24/12
		
		if (humanControl->GearShiftDesired()) {
					//robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "GearShift Desired" );
					//robot->dsLCD->UpdateLCD();
					printf("Shifting gear\n");
					robot->ShiftGear();
				}
		
/*		//Going onto Ramp by putting down the ramp
				if (humanControl->RampManipDesired()) {
					//robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Ramp Desired" );
					//robot->dsLCD->UpdateLCD();
					printf("Toggling ramp manipulator\n");
					robot->ManipRamp();
				}*/
		
		
		if (humanControl->RampManipDesired()) {
			if (robot->RampArmIsUp()) {
					//robot->LowerRampLegs(); // commented up to create separate button to put out legs. 
					intakeLoweringDelayStart = currTimeSec;
					printf("beginning delay\n");
					//robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Ramp manip lowered" );
					//robot->dsLCD->UpdateLCD();
				} else {
					//robot->RaiseRampLegs(); // commented out to create separateness
					robot->RaiseIntake();
					//robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Ramp manip raised" );
					//robot->dsLCD->UpdateLCD();
				}
		}
				//Delay between command to lowering ramp legs and lower intake so that legs are fully extended before 
				if (intakeLoweringDelayStart != 0 && currTimeSec > (intakeLoweringDelayStart + intakeLoweringDelay)) {
					printf("delay finished\n");
					robot->LowerIntake();
					intakeLoweringDelayStart = 0.0;
				}
	
		if (humanControl->RampLegsDesired()) {
			if (robot->RampLegsAreUp()) {
				robot->LowerRampLegs();
				printf("Lowering Ramp Legs\n");
			} else {
				robot->RaiseRampLegs();
				printf("Raising Ramp Legs\n");
			}
		}
		

#endif
		
		if (humanControl->TargetPivotDesired()){
			//cameraController->RequestNewCameraImage();
			TurnToTopTargetAngle(0.8);
		}
		
		// DemoAutoDrive for ISS testing - DO NOT MODIFY!!
		if (humanControl->DemoAutoDriveDesired()) {
			RequestAutoDrive(5, 0.7);
		}
		
		if (humanControl->DemoAutoPivotDesired()) {
			RequestAutoPivot(15, 0.7);
		}
		
		if (humanControl->BrakeToggleDesired()) {
			//robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "BRAKE Desired" );
			//robot->dsLCD->UpdateLCD();
			printf("Toggling brake\n");
			robot->ToggleBrake();
		}
	
		if (autoDriveRequested) {
			// first disengage brakes if they are on
			robot->SetBrakeOff();
			printf("autoDriveRequested is true, setting next state to kAutoDriveStart\n");
			nextState = kAutoDriveStart;
		} else if (autoPivotRequested) {
			nextState = kAutoPivotStart;
		}else {
			nextState = kTeleopDrive; //stays in teleop
		}

	break;

	case (kAutoDriveStart):
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "AutoDrive Start" );
		robot->dsLCD->UpdateLCD();
		robot->SetWheelSpeed(0.5, RobotModel::kBothWheels);
		Drive_sum_error = 0.0;
		printf("Starting auto drive\n");
		nextState = kAutoDrive;
	break;

	case (kAutoDrive):
			
		// Set these values at the beginning, so we can refer to them over and over and have the same value.
		rightWheelEncoderDistance = robot->GetWheelEncoderDistance(RobotModel::kRightWheel);
		leftWheelEncoderDistance = robot->GetWheelEncoderDistance(RobotModel::kLeftWheel);
		
		// PID loop works more easily when we work with only positive distances
		autoDistTraveled = (fabs(rightWheelEncoderDistance) + fabs(leftWheelEncoderDistance))/2.0;
		
		// Directional correction
		currAngle = robot->GetGyroAngle();
		float leftSpeedAdjust = (currAngle - autoInitialGyroAngle) * (DrivePID_Gyro_P);
		//float leftSpeedAdjust = DrivePID_Encoder_P * (leftWheelEncoderDistance - rightWheelEncoderDistance);
		// DrivePID_Encoder_P is a negative number
		if(fabs(leftSpeedAdjust) > 0.3) leftSpeedAdjust = (leftSpeedAdjust > 0) ? 0.3: -0.3 ;

		currSpeedFactor = DrivePID(autoGoalDist, autoDistTraveled, &autoDiffDriveError);
		// PID override - CHECK THIS POLARITY INVERSION ...!!!!
		/*if(autoGoalDist < 0.0)*/ leftSpeedAdjust = -leftSpeedAdjust ; // CHECK THIS AFTER SVR
		// Adjust speed with PID factor
		currSpeed = autoSpeed * currSpeedFactor ;
		robot->SetWheelSpeed( (currSpeed + leftSpeedAdjust) , RobotModel::kLeftWheel);
		robot->SetWheelSpeed( (currSpeed - leftSpeedAdjust) , RobotModel::kRightWheel);

		if ( (fabs(autoDistTraveled) >= fabs(autoGoalDist)) ) { // && 
				//(fabs(currSpeedFactor < 0.25)) && autoDiffDriveError < 0.001 ){	// PID loop has settled
			
			printf("Drive done (by distance) at distance %f\n",autoDistTraveled);
			nextState = kAutoDriveDone; //if you've traveled the requested distance, go to end of auto
			
		}  else if (fabs(autoDiffDriveError) < 0.005) {
			if (pidExitCountdownStartTime == 0.0) {
				pidExitCountdownStartTime = currTimeSec;
				printf("Starting drive PID exit countdown timer.\n");

			}
			if ( (pidExitCountdownStartTime - currTimeSec) >= pidExitDelay) {
				pidExitCountdownStartTime = 0.0;
				nextState = kAutoDriveDone;
				printf("AutoDrive stalled - Timer expired - Done at %f\n", autoDistTraveled);
			}
	
		} else {
			
			// not there yet!
			DO_PERIODIC(25, printf("Not stopping because autoDiffDriveError = %f\n", autoDiffDriveError));
			DO_PERIODIC(25, printf("Raw encoder R: %f L: %f\n", robot->GetWheelEncoderValue(RobotModel::kRightWheel),
						robot->GetWheelEncoderValue(RobotModel::kLeftWheel)));
			DO_PERIODIC(25, printf ("LSA: %f currspeed : %f angle: %f dist traveled: %f, goal dist: %f\n", leftSpeedAdjust, currSpeed, currAngle, autoDistTraveled, autoGoalDist));
			DO_PERIODIC(25, printf ("currSpeedFactor: %f  autoDiffDriveError: %f\n", currSpeedFactor, autoDiffDriveError));
			nextState = kAutoDrive;
			
		}
		break;

	case kAutoDriveDone:
		autoDriveRequested = false;
		autoDriveDone = true;

		robot->SetWheelSpeed(0, RobotModel::kRightWheel); //stop here - paw
		robot->SetWheelSpeed(0, RobotModel::kLeftWheel);

		robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "AutoDrive Done" );
		robot->dsLCD->UpdateLCD();
		
		printf("Stopped driving at encoder distance: %f", robot->GetWheelEncoderValue(RobotModel::kRightWheel));

		nextState = kTeleopDrive;//once done with autonomous drive, go to idle
		break;

	case (kAutoPivotStart):
		// stuff you want to do once at the beginning of a pivot turn
				
		//if not in low gear, shift to low gear before pivoting
		/****
		if (!(robot->IsLowGear())) {
			originallyHighGear = true;
			robot->ShiftGear();
			printf("shifting to low gear to start pivot");
		}
		else {
			printf("Not engaging gear shifter because robot is originally in low gear.");
		}
		*******/

		Pivot_sum_error = 0.0;
		nextState = kAutoPivot;
		
		printf("in kAutoPivotStart curr abs: %f target : %f\n", autoInitialGyroAngle, autoPivotGoal);
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "AutoPivot: from %f deg to %f", 
														autoInitialGyroAngle, (autoPivotGoal));
		robot->dsLCD->UpdateLCD();
		
		break;
	
	case (kAutoPivot):
			
		currAngle = robot->GetGyroAngle();			// clockwise = positive
		pivotSpeedFactor = PivotPID(autoPivotGoal, currAngle, &autoDiffPivotError);		//  comment out for no PID;
		adjustedAutoPivotSpeed = autoPivotSpeed * pivotSpeedFactor ;	// comment out for no PID

		robot->SetWheelSpeed(adjustedAutoPivotSpeed, RobotModel::kLeftWheel);
		robot->SetWheelSpeed(-adjustedAutoPivotSpeed, RobotModel::kRightWheel); // note the negative sign
		
		// DO_PERIODIC(20, printf("currTime = %f, DeltaTime = %f\n", currTimeSec, deltaTimeSec));
		// DO_PERIODIC(25, printf("currAngle = %f, goalAngle = %f, rightSpeed = %f, leftSpeed = %f\n", currAngle, autoPivotGoal, adjustedAutoPivotSpeed, -adjustedAutoPivotSpeed));
		
		/**
		if (m_CurrentGyroValue > m_RequestedPivotAngle) {
			// need to turn ccw: left motor back, right motor forward
			m_pRobotModel->SetWheelMotorValue(RobotModel::kRightWheel, m_RequestedDriveSpeed);
			m_pRobotModel->SetWheelMotorValue(RobotModel::kLeftWheel, -m_RequestedDriveSpeed);	// note the negative sign
		}
		else if (m_CurrentGyroValue < m_RequestedPivotAngle) {
			// need to turn cw: right motor back, left motor forward
		 	m_pRobotModel->SetWheelMotorValue(RobotModel::kRightWheel, -m_RequestedDriveSpeed);	// note the negative sign
		 	m_pRobotModel->SetWheelMotorValue(RobotModel::kLeftWheel, m_RequestedDriveSpeed);	
		}
		**/
		
		if (fabs(currAngle - autoPivotGoal) < 0.5){
			nextState = kAutoPivotDone;
			printf("AutoPivot Done at %f\n", currAngle);
	  //} else if ( (fabs(currAngle - autoPivotGoal) < 3.0 ) && (autoDiffPivotError == 0.0)){
		} else if ( fabs(autoDiffPivotError) < 0.0001 ) {
			if (pidExitCountdownStartTime == 0.0) {
				pidExitCountdownStartTime = currTimeSec; // Start timer
				printf("Starting pivot PID exit countdown timer.\n");
			}
			if ( (currTimeSec - pidExitCountdownStartTime) >= pidExitDelay ) {
				pidExitCountdownStartTime = 0.0; // reset timer
				nextState = kAutoPivotDone; //if you've NOT turned the requested angle, but don't move for 1 sec
				printf("AutoPivot stalled - Timer expired - Done at %f\n", currAngle);
			}
		} else { // fabs(currAngle - autoPivotGoal) >= 0.5  and  autoDiffPivotError >= 0.0001
			pidExitCountdownStartTime = 0.0; // reset timer
		}
		
		//DO_PERIODIC(20, printf("in AutoPivot\n"));

		break;
	
	case (kAutoPivotDone):
		autoPivotRequested = false;
		autoPivotDone = true;
		robot->SetWheelSpeed(0.0, RobotModel::kBothWheels);	//stop here - paw
		if (originallyHighGear) {
			robot->ShiftGear();
			printf("Shifting back to original high gear");
		}
		else{
			printf("Was originally in low gear, so did not engage shifter.");
		}
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 30, "AutoPivot Done" );
		robot->dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Final Angle %f", robot->GetGyroAngle());
		robot->dsLCD->UpdateLCD();
		//shift back to high gear
		nextState = kTeleopDrive;					//once done with autonomous pivot, go to idle
		break;
		
	}
	
	m_stateVal = nextState; 
	
}


double DriveController::DrivePID(double goalDist, double distTraveled, double *diffDriveError) {
	static int count = 0;
	static float last_error = 0.0 ;
	float PFac = DrivePID_P;
	float IFac = DrivePID_I;
	float DFac = DrivePID_D;
	float NError = goalDist  - distTraveled ;
	
	// more than N feet - saturate
	// also means we start deceleration with N feet to go
	if(NError > 3.0) NError = 3.0;

	double pterm = PFac * NError;
	double difErr = (NError - last_error);
	
	Drive_sum_error += NError ;
	// saturation of the I term
	Drive_sum_error = (Drive_sum_error > (1.0/IFac)) ? (1.0/IFac) : Drive_sum_error ;
	Drive_sum_error = (Drive_sum_error < (-1.0/IFac)) ? (-1.0/IFac) : Drive_sum_error ;
	
	double iterm = IFac * Drive_sum_error ;
	
	DO_PERIODIC(25, printf("**difErr = %f IErr = %f NError = %f\n",difErr, Drive_sum_error, NError));
	
	if(difErr >= fabs(NError)) difErr = 0.0 ; // account for first time


	double dterm = DFac * (difErr) ;
	double motorValue = pterm + iterm + dterm;

	// DO_PERIODIC(25, printf("** pterm = PFac * NError = %f * %f = %f\n", PFac, NError, pterm));
	// DO_PERIODIC(25, printf("** dterm = DFac * difErr = %f * %f = %f\n", DFac, difErr, dterm));
	// DO_PERIODIC(25, printf("** motorValue = p + i + d = %f + %f +%f = %f\n", pterm, iterm, dterm, motorValue));
	
	//saturation of the terms
	if (motorValue > 0.0) {motorValue = min(motorValue, 1.0);}
	else {motorValue = max(motorValue, -1.0);}

	if (count++ % 10 == 0) {
		printf("Wheel Motor On: %f err: %f difErr: %f pt: %f it: %f dt: %f \n", motorValue, NError,difErr, pterm, iterm, dterm);
	}
	//printf("distance traveled = %f\n",m_feetTraveled);}
	last_error = NError;
	(*diffDriveError) = difErr;

	return 	motorValue;
}

double DriveController::PivotPID(double goalAngle, double currentAngle, double *diffAngleError) {
	
	static float last_error = 0.0 ;
	float NError =  currentAngle - goalAngle;
	
	// more than N feet - saturate
	// also means we start deceleration with N feet to go
	if(NError > 10.0) NError = 10.0;
	if(NError < -10.0) NError = -10.0 ;
	
	float pterm = PivotPID_P * NError;
	
	Pivot_sum_error += NError ;
	// saturation of the I term
	Pivot_sum_error = (Pivot_sum_error > (1.0/PivotPID_I)) ? (1.0/PivotPID_I) : Pivot_sum_error ;
	Pivot_sum_error = (Pivot_sum_error < (-1.0/PivotPID_I)) ? (-1.0/PivotPID_I) : Pivot_sum_error ;
	
	float iterm = PivotPID_I * Pivot_sum_error ;
	
	float difErr = (NError - last_error);
	if(difErr >= NError) difErr = 0.0 ; // account for first time


	float dterm = PivotPID_D * difErr;
	double motorValue = pterm + iterm + dterm;
	
	// saturation of the terms
	if (motorValue > 0.0) {motorValue = min(motorValue, 1.0);}
	else {motorValue = max(motorValue, -1.0);}

	DO_PERIODIC(25, printf("Wheel Pivot Curr: %f Goal %f\n", currAngle,goalAngle));
	DO_PERIODIC(25, printf("Wheel Pivot @ %f: motorVal: %f err: %f difErr: %f Sum_err: %f pt: %f it: %f dt: %f \n", currAngle, motorValue, NError, difErr, Pivot_sum_error, pterm, iterm, dterm));

	last_error = NError;
	(*diffAngleError) = difErr ;

	return 	motorValue;
}

void DriveController::RequestAutoDrive(double distance, double speed) {
	autoDriveRequested = true;
	autoDriveDone = false;
	autoGoalDist = distance;
	autoSpeed = speed;
	autoDistTraveled = 0.0;
	Drive_sum_error = 0.0;
	autoInitialGyroAngle = robot->GetGyroAngle();
	robot->ResetWheelEncoder();
	printf("Request auto drive, distance: %f speed: %f\n", autoGoalDist, autoSpeed);
}
void DriveController::RequestAutoPivot(double target_angle, double speed) {
	autoPivotRequested = true;
	autoPivotDone = false;
	autoPivotSpeed = speed;
	Pivot_sum_error = 0.0;
	autoInitialGyroAngle = robot->GetGyroAngle();
	autoPivotGoal = autoInitialGyroAngle + target_angle;

}

void DriveController::Reset() {
	mustReset = true;
	Drive_sum_error = Pivot_sum_error = 0.0;

}

void DriveController::RefreshIni(){
	DrivePID_P = (robot->pini)->getf("DrivePID", "PFAC", 0.4);
	DrivePID_I = (robot->pini)->getf("DrivePID", "IFAC", 0.0);
	DrivePID_D = (robot->pini)->getf("DrivePID", "DFAC", 11.0);
	DrivePID_Gyro_P = (robot->pini)->getf("DrivePID", "GYRO_PFAC", -0.0155);
	DrivePID_Encoder_P = robot->pini->getf("DrivePID", "ENCODER_PFAC", -2.13);
		
	PivotPID_P = robot->pini->getf("PivotPID", "PFAC", 0.15);
	PivotPID_I = robot->pini->getf("PivotPID", "IFAC", 0.0);
	PivotPID_D = robot->pini->getf("PivotPID", "DFAC", 3.0);
		
	Drive_sum_error = Pivot_sum_error = 0.0;
	
	printf("INI file... DrivePID: PFAC = %f, IFAC = %f DFAC = %f, Gyro_P = %f, Encoder_P = %f\n", DrivePID_P, DrivePID_I, DrivePID_D, DrivePID_Gyro_P, DrivePID_Encoder_P);
	printf("INI file... PivotPID: PFAC = %f, IFAC = %f DFAC = %f\n", PivotPID_P, PivotPID_I, PivotPID_D );

}

bool DriveController::AutoDriveDone() {
	return autoDriveDone;
}

bool DriveController::AutoPivotDone() {
	return autoPivotDone;
}
// create function that gets the angle to the topTarget from Camera Controller
// gets the robot to move that angle

void DriveController::TurnToTopTargetAngle(double speed){
	double angle;
#ifdef USE_CAMERA
	angle = cameraController->GetAngleToTopTarget();
#else
	angle = 0;
#endif
	if (angle == 0){		// angle = 10.0;
		printf("Did not find valid top target, so pivoting 0 degrees \n");
	}
	RequestAutoPivot(angle, speed); 
}

double DriveController::GetIntakeLoweringDelay() {
	return intakeLoweringDelay;
}

DriveController::~DriveController()
{
}
