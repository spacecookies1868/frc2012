#include "ISS.h"

ISS::ISS(RobotModel* myRobot, RemoteController* myHumanControl, DriveController* myDriveControl,
		ShooterController* myShooterControl){
	robot = myRobot;
	humanControl = myHumanControl;
	driveControl = myDriveControl;
	shooterControl = myShooterControl;
}
void ISS::Update(double currTimeSec, double deltaTimeSec) {
	
	switch(m_stateVal){
	
	case (kIdle):
		
		if (humanControl->IssDriveForwardDesired()){
			nextState = kDriveForward;
		}
		
		if (humanControl->IssAimDesired()) {
			nextState = kAim;
		}
		
		if (humanControl->IssShootDesired()) {
			nextState = kShoot;
		}
		break;
	  
	case (kDriveForward):
	   
		driveControl->RequestAutoDrive(2, .2);
		nextState = kDriveForwardDone;
		
		break;
	    
	case (kDriveForwardDone):
		
		if (driveControl->AutoDriveDone()){
			robot->SetWheelSpeed(0.0, robot->kBothWheels);
			nextState = kIdle;
		}
		else {
			nextState = kDriveForwardDone;
		}
		  break;
		
	case (kAim):
	   
		driveControl->TurnToTopTargetAngle(.25);
	
		nextState = kAimDone;
	      break;
	
	case (kAimDone):
			
		if(driveControl->AutoPivotDone()){
			robot->SetWheelSpeed(0.0, robot->kBothWheels);
			nextState = kIdle;
		}		
		else {
			nextState = kAimDone;
		}
		  break;
	
	case (kShoot):
			
		nextState = kIdle;
		
		
		break;
	
	case (kReset):
			
		
		break;
	
	}
	
	m_stateVal = nextState;

}

ISS::~ISS()
{
}
