#include <math.h>
#include "ShooterController.h"
#include "RobotModel.h"
#include "Debugging.h"
#define FILTER_SIGNALS 1

ShooterController::ShooterController(RobotModel *myRobot, RemoteController *myHumanControl) {
		robot = myRobot;
		humanControl = myHumanControl;
		m_stateVal = kInitialize;
		m_FlyWheelSpeed = 0.0 ;
		
		m_InitialFlywheelEncoderVal = 0.0;
		m_InitialFlywheelTime = 0.0;
	
		m_MinFlywheelEncoderChange = 7.0; //IDK
		m_FlywheelStallTime = 1.5; //IDK
		
		m_RollerSpeed = 1.0;
		m_DefaultConveyorSpeed = 0.9;
		m_ConveyorGoalDistance = 0.0;
		advanceBallStopRollerTimerBegin = 0.0;
		advanceBallConveyorStartTimerBegin = 0.0;
		
		advanceBallEncoderTicks = 0;
		
		m_bConveyorDone = true; 
		m_bReverseConveyorDone = true;
		
		m_bRollerMotorsOn = false; 
		m_bRollerMotorsOnReverse = false;//*
		m_bTogglePickup = false;
		m_bToggleReversePickup = false; //*HELLLLLOOOO HANSAAAAA
		m_bAdvanceBall = false;
		
		m_bRequestConveyor = false;
		m_bRequestReverseConveyor = false;
		m_bAdvanceBallRequested = false;
		
		m_bRequestJogConveyor = false;// button
		m_bSensorRequestedAdvance = false;//sensor	
		m_bRequestToggleConveyor = false;// button 
		
		m_bRequestInfiniteConveyor = false;
		
		m_bRequestHoodAdjust = false; 
		
		m_bRequestShooter = false;
		m_bFlywheelDesiredOn = false;
		m_bNeedToMoveUpSingleBall = false;
		m_FlywheelMotorVal = 0.0;
		m_FlywheelSpeedAdjustMultiplier = 0.0;
		m_bHumanRequestsFlywheelOff = false;
		m_ConveyorToggleState = 0;		
		m_bNeedToTurnOffRollers = false;
		m_bIntakeToggleDesired = false;
		
		mustReset = false;
		
		// These values are strictly for testing and will be changed
		RefreshIni();

		m_DesiredFlywheelSpeed = m_FlywheelSpeedFender;
		
		m_last_time = robot->timer->Get();
		m_last_value = 0;
		
		ResetFilters();

}

void ShooterController::ResetFilters()
{
	for (int i = 0; i < IP_FILTER_SIZE; i++) {
		m_ifilter[i] = 0.0;
	}
	m_ifilterindex = 0;
	for (int i = 0; i < OP_FILTER_SIZE; i++) {
		m_ofilter[i] = 0;
	}
	m_ofilterindex = 0;
	
}

// take a new value in - filter it and return filtered value
// simple averaging filters of FILTER_SIZE
// can be templated as an exercise ...
double ShooterController::InputFilter(double newvalue){
	double out = 0.0;
	m_ifilter[m_ifilterindex] = newvalue;
	m_ifilterindex++ ;
	if (m_ifilterindex == IP_FILTER_SIZE) {
		m_ifilterindex = 0;
	}
	for(int i = 0; i < IP_FILTER_SIZE; i++){
		out += m_ifilter[i];
	}
	return (out/(double)IP_FILTER_SIZE);
}

double ShooterController::OutputFilter(double newvalue){
	double out = 0.0;
	m_ofilter[m_ofilterindex] = newvalue;
	m_ofilterindex++ ;
	if (m_ofilterindex == OP_FILTER_SIZE) {
		m_ofilterindex = 0;
	}
	for(int i = 0; i < OP_FILTER_SIZE; i++){
		out += m_ofilter[i];
	}
	return (out/(double)OP_FILTER_SIZE);

}

void ShooterController::Update(double currTimeSec, double deltaTimeSec) {

		m_curr_time = currTimeSec ;
		m_CurrentFlywheelEncoderVal = robot->shooterEncoder->Get();
		m_bTogglePickup = humanControl->PickUpToggleDesired();
		// m_bRequestJogConveyor = humanControl->JogConveyorDesired();
		m_bRequestToggleConveyor = false; // = humanControl->ToggleConveyorDesired();
		m_bHumanRequestsFlywheelOff = humanControl->FlywheelDesiredOff();
		m_bRequestShooter = humanControl->ShootingDesired(); 
		m_bRequestReverseConveyor = false; // = humanControl->ReverseConveyorDesired();
		m_bPreviousBallIntakeSensed = m_bBallIntakeSensed;
		m_bBallIntakeSensed = robot->IntakeSensorActivated(); // = false;  // set this to FALSE to disable sensor
		m_bRequestHoodAdjust = humanControl->HoodAdjustDesired();
		m_FlywheelSpeedAdjustMultiplier = humanControl->GetFlywheelSpeedAdjustMultiplier();
		m_bToggleReversePickup = humanControl->ToggleReverseRollersDesired();
		m_bAdvanceBallRequested = humanControl->AdvanceBallDesired();
		m_ConveyorToggleState = humanControl->ConveyorToggleState();
		m_bIntakeToggleDesired = humanControl->IntakeToggleDesired();
		
		//robot->dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "flywheelAdjust %f", m_FlywheelSpeedAdjustMultiplier);
		
		m_LastShootingPosition = m_ShootingPosition;
		m_ShootingPosition = humanControl->ShootingPosition();
		m_bShootingPositionChanged = (m_LastShootingPosition != m_ShootingPosition);
		

		// Modify below toggle - once we have new DS - with 2 buttons for each position
		// The FlyWheel speed can be set/read every update - but the hood should be set
		// only if the input was changed/activated.
		if (m_bRequestHoodAdjust) {
			robot->ToggleHood();
		}
		
		// Set conveyor forward/backwards depending on the position of the toggle
		if (m_ConveyorToggleState > 0) {
			robot->SetConveyorSpeed(m_DefaultConveyorSpeed);
			// DO_PERIODIC(50, printf("conveyor up\n"));
		} else if (m_ConveyorToggleState < 0) {
			robot->SetConveyorSpeed(-m_DefaultConveyorSpeed);
			m_bRollerMotorsOnReverse = true;
			m_bNeedToTurnOffRollers = true;
			// DO_PERIODIC(50, printf("conveyor down\n"));
		} else {
			robot->SetConveyorSpeed(0.0);
			if (m_bNeedToTurnOffRollers) {
				m_bRollerMotorsOnReverse = false;
				m_bNeedToTurnOffRollers = false;
			}
			// DO_PERIODIC(50, printf("conveyor off\n"));
		}
		
		
		//if(robot->IsHoodLowered()) m_DesiredFlywheelSpeed = m_FlywheelSpeedFender;
		//else m_DesiredFlywheelSpeed = m_FlywheelSpeedKey;

		if (m_bShootingPositionChanged) {
			if (m_ShootingPosition == kFender) {
				m_DesiredFlywheelSpeed = m_FlywheelSpeedFender;
				m_bFlywheelDesiredOn = true;
				robot->LowerHood();
				printf("Shooting from FENDER\n");
		        robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "shoot FENDER" );
		    	robot->dsLCD->UpdateLCD();
		    	m_InitialFlywheelTime = m_curr_time;
		    	m_InitialFlywheelEncoderVal = m_CurrentFlywheelEncoderVal;
			} else if (m_ShootingPosition == kKeyNear) {
				m_DesiredFlywheelSpeed = m_FlywheelSpeedNearKey;
				robot->RaiseHood();
				m_bFlywheelDesiredOn = true;
				printf("Shooting from NEAR KEY\n");
				robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "shoot NEAR KEY" );
				robot->dsLCD->UpdateLCD();
				m_InitialFlywheelTime = m_curr_time;
				m_InitialFlywheelEncoderVal = m_CurrentFlywheelEncoderVal;
			} else if (m_ShootingPosition == kKeyFar) {
				m_DesiredFlywheelSpeed = m_FlywheelSpeedFarKey;
				robot->RaiseHood();
				m_bFlywheelDesiredOn = true;
				printf("Shooting from FAR KEY\n");
				robot->dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "shoot FAR KEY" );
				robot->dsLCD->UpdateLCD();
				m_InitialFlywheelTime = m_curr_time;
				m_InitialFlywheelEncoderVal = m_CurrentFlywheelEncoderVal;
			}
		}
		
		 if(m_bAdvanceBallRequested){
			 // turn the rollers on for 1.5 seconds
			 /* goes to finite conveyor state in switch statement and runs
			  * conveyor for set distance (to be determined)
			  * then from there goes to the idle state
			*/
			 m_bRollerMotorsOn = true;
			 m_bAdvanceBall = true;
			 advanceBallStopRollerTimerBegin = currTimeSec;
 			 advanceBallConveyorStartTimerBegin = currTimeSec;
		 }
		 
		 
		 if (advanceBallStopRollerTimerBegin > 0
				 && currTimeSec > advanceBallStopRollerTimerBegin + 1.5 ) {
			 m_bRollerMotorsOn = false;
			 advanceBallStopRollerTimerBegin = 0;
		 }
		 
		 // If the driver pressed the roller on/off button, turn the rollers to the opposite
		 // of whatever they are right now
		 if(m_bTogglePickup){
			 m_bRollerMotorsOn = !m_bRollerMotorsOn; 
			 printf("toggling rollers\n");
		 }	
		 
		 // Raise/lower the intake mechanism
		 if (m_bIntakeToggleDesired) {
			 robot->ToggleIntake();
		 }
		 
		 // Turn off the rollers if a ball passes the sensor
		 if (!m_bPreviousBallIntakeSensed && m_bBallIntakeSensed){
		 	m_bRollerMotorsOn = false;
		 }
		 
		 
		 if(m_bToggleReversePickup) {							//*The next few lines are sabs
			 m_bRollerMotorsOnReverse = !m_bRollerMotorsOnReverse;
		 }
		 
		 if(m_bRollerMotorsOn){
			 robot->SetRollerSpeed(m_RollerSpeed);
			 m_bRollerMotorsOnReverse = false;
			 robot->dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "rollers FORWARD                ");
			 robot->dsLCD->UpdateLCD();
		 } else if(m_bRollerMotorsOnReverse) {
			 robot->SetRollerSpeed(-(m_RollerSpeed));
			 m_bRollerMotorsOn = false;
			 robot->dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "rollers REVERSE                ");
			 robot->dsLCD->UpdateLCD();
		 } else {
			 robot->SetRollerSpeed(0.0);
			 robot->dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "rollers OFF                ");
			 robot->dsLCD->UpdateLCD();
		 }
		 		 
		 
		 if(m_bRequestShooter){
			 m_bFlywheelDesiredOn = !m_bFlywheelDesiredOn;
			// printf("turning on flywheel b/c operator pressed  button %f.\n", m_curr_time);
			 
			
		 }
		 
		 if (m_bHumanRequestsFlywheelOff) {
			 m_bFlywheelDesiredOn = false;
			 printf("turning off flywheel b/c operator pressed preset button again.\n");
		 }
		 
		 
		 if (m_bFlywheelDesiredOn) {
			// printf("InitialEncoder: %f, CurrEncoder: %f, InitialTime: %f, CurrTime: %f\n",
				//	 m_InitialFlywheelEncoderVal, m_CurrentFlywheelEncoderVal, m_InitialFlywheelTime, m_curr_time);
			 if (((m_curr_time - m_InitialFlywheelTime) > m_FlywheelStallTime) && 
					 ((m_CurrentFlywheelEncoderVal - m_InitialFlywheelEncoderVal) 
							 < m_MinFlywheelEncoderChange)) {
			//	 printf("Flywheel desired on being set false - flywheel shouldn't turn on.\n");
				 m_bFlywheelDesiredOn = false;
				// m_InitialFlywheelTime = m_curr_time;
				// m_InitialFlywheelEncoderVal = m_CurrentFlywheelEncoderVal;
			 }
		 }
		 
		 
		 if(m_bFlywheelDesiredOn){

			 
			 double difError = 0.0;
			 double speedFactor = 1.0;
			 double desired = (m_DesiredFlywheelSpeed * m_FlywheelSpeedAdjustMultiplier);
			 static int cnt = 0;
			 m_FlywheelMotorVal = .98 ;
			 m_CurrentFlywheelSpeed = GetFlywheelSpeed();
#if FILTER_SIGNALS
			 m_CurrentFlywheelSpeed = InputFilter(m_CurrentFlywheelSpeed);
#endif

			 speedFactor = FlywheelPID((desired), m_CurrentFlywheelSpeed, &difError);
			 m_FlywheelMotorVal = speedFactor ;
#if FILTER_SIGNALS
			 m_FlywheelMotorVal = OutputFilter(m_FlywheelMotorVal);
#endif
			 
			 DO_PERIODIC(20, printf("** Time: %f err: %f desired: %f motor: %f\n", currTimeSec, (desired - m_CurrentFlywheelSpeed), (desired),  m_FlywheelMotorVal));

			 robot->SetFlywheelSpeed(m_FlywheelMotorVal);
			 
			 if(cnt++ % 10) {
				 // we really shouldnt print every update - that causes performance issues.
				 robot->dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "FW c %d d %d        ", (int)(m_CurrentFlywheelSpeed), (int)(m_DesiredFlywheelSpeed * m_FlywheelSpeedAdjustMultiplier) );
				 // robot->dsLCD->UpdateLCD();
			 }
		 }
		 else {
			 robot->SetFlywheelSpeed(0.0);
			 m_Flywheel_sum_error = 0.0;
			 robot->dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "flywheel OFF                ");
			 robot->dsLCD->UpdateLCD();
		 }
		 
		 if (mustGoToIdle) {
			 m_stateVal = kIdle;
			 mustGoToIdle = false;
		 }
		 
		 if (mustReset) {
			 m_stateVal = kReset;
			 mustReset = false;
		 }
		
		 switch (m_stateVal) {
		 case (kReset):
			robot->RaiseHood();
		 	robot->SetFlywheelSpeed(0.0);
		 	robot->SetConveyorSpeed(0.0);
		 	robot->SetRollerSpeed(0.0);
		 	
		 	m_ConveyorGoalDistance = 0.0;
		 			
			m_bConveyorDone = true; 
			m_bReverseConveyorDone = true;
			
			m_bRollerMotorsOn = false; 
			m_bRollerMotorsOnReverse = false;//*
			m_bTogglePickup = false;
			m_bToggleReversePickup = false; //*HELLLLLOOOO HANSAAAAA
			
			m_bRequestConveyor = false;
			m_bRequestReverseConveyor = false;
			m_bAdvanceBall = false;
			
			m_bRequestJogConveyor = false;// button
			m_bSensorRequestedAdvance = false;//sensor	
			m_bRequestToggleConveyor = false;// button 
			
			m_bRequestInfiniteConveyor = false;
			
			m_bRequestHoodAdjust = false; 
			
			m_bRequestShooter = false;
			m_bFlywheelDesiredOn = false;
			m_bNeedToMoveUpSingleBall = false;
			m_bHumanRequestsFlywheelOff = false;
			m_FlywheelMotorVal = 0.0;
			m_Flywheel_sum_error = 0.0;
			m_last_value = robot->shooterEncoder->Get();
			
			ResetFilters();
			
		 	
		 	nextState = kIdle; 
		 		 
		 	break;
		 	
		 case (kInitialize):
				 
		 	nextState = kIdle;
		 		 
			break;
				
		 case (kIdle):
				 
			// DO_PERIODIC(1000, printf("RequestInfiniteConveyor is %s\n", m_bRequestInfiniteConveyor ? "true" : "false"));
		 
			m_bConveyorDone = true; 
			//robot->SetConveyorSpeed(0.0);		 
		
			/*
			if (m_bRequestJogConveyor) {
				printf("Request Jog Conveyor true\n");
				m_InitialConveyorEncoderValue = robot->GetConveyorEncoderValue();
				m_ConveyorGoalDistance = 256; 
				nextState = kAdvanceConveyorFinite;
			}			
			else */
			if(m_bAdvanceBall){
				m_InitialConveyorEncoderValue = robot->GetConveyorEncoderValue();
				m_ConveyorGoalDistance = advanceBallEncoderTicks; 
				m_bAdvanceBall = false;
				printf("bAdvanceBall was true curr: %f goal: %f\n", m_InitialConveyorEncoderValue, m_ConveyorGoalDistance );
				nextState = kAdvanceConveyorFinite;
			}
			else if (m_bRequestToggleConveyor || m_bRequestInfiniteConveyor) {
				printf("moving to infinite conveyor because RequestInfiniteConveyor is true\n");
				nextState = kAdvanceConveyorInfinite;
			}
			else if (m_bRequestReverseConveyor) {
				printf("moving to reverse conveyor because RequestReverseConveyor is true\n");
				nextState = kReverseConveyor;
			}
			else {
				nextState = kIdle;
			}
			 
			break;
			
		 case (kAdvanceConveyorFinite):		
			/*printf("[In kAdvanceConveyorFinite] DiffConvEncVal=[%f] ConveyorGoalDist=[%f] ConveyorEncoderValue=[%f] InitialConveyorEncorderValue=[%f]\n",
							m_DiffConveyorEncoderValue, m_ConveyorGoalDistance, m_ConveyorEncoderValue, m_InitialConveyorEncoderValue  );*/
			m_bConveyorDone = false; 
			robot->SetConveyorSpeed(m_DefaultConveyorSpeed);
				 	
			//Calculates difference between where the encoder was originally when
			//first going into state and the location of encoder when it cycles
			//through again if the desired difference is not met. 	 	
			m_ConveyorEncoderValue = robot->GetConveyorEncoderValue();
			m_DiffConveyorEncoderValue = fabs(m_ConveyorEncoderValue - m_InitialConveyorEncoderValue);
			// DO_PERIODIC(25, printf("Curr Enc: %f Initial = %f\n", m_ConveyorEncoderValue, m_InitialConveyorEncoderValue ));
		 			
			if (m_DiffConveyorEncoderValue > m_ConveyorGoalDistance || m_bRequestToggleConveyor){
				nextState = kIdle;
			}
			break;
			
		 case (kAdvanceConveyorInfinite):
			m_bConveyorDone = false;	
		 	 
			robot->SetConveyorSpeed(m_DefaultConveyorSpeed);	 
			
			/*
			if (m_bRequestJogConveyor) {
				m_InitialConveyorEncoderValue = robot->GetConveyorEncoderValue();
				m_ConveyorGoalDistance = 256; 
				printf("exiting infinite conveyor because jog\n");
				nextState = kAdvanceConveyorFinite;
			}
			else */
			if (m_bRequestToggleConveyor) {
				printf("exiting infinite conveyor because toggle\n");
				nextState = kIdle;
			}
			else {
				nextState = kAdvanceConveyorInfinite;
			}
		 	
			break;			
		 case (kReverseConveyor):
			 // printf("in ReverseConveyor\n");
		 
		 
			 m_bReverseConveyorDone = false; 
			 robot->SetConveyorSpeed(-m_DefaultConveyorSpeed);
			 robot->SetRollerSpeed(-m_RollerSpeed);							

			if (m_bRequestReverseConveyor) {
				printf("exiting reverse conveyor because toggle\n");
				nextState = kReverseConveyorDone;
			}
			else {
				nextState = kReverseConveyor;
			}


							
			break;
		 case (kReverseConveyorDone):
			printf("in ReverseConveyorDone\n");
		 	m_bReverseConveyorDone = true; 
			m_bRequestReverseConveyor = false; 
		 	robot->SetConveyorSpeed(0.0);
			robot->SetRollerSpeed(0.0);

					 	
		 	nextState = kIdle;
			break;		 	
		 
		 }
		 
		 m_stateVal = nextState;  
}

		


 void ShooterController::SetShooterSpeed(double speed){ m_DesiredFlywheelSpeed = speed;}
 void ShooterController::SetShooterSpeedByPosition(int pos) {
	 switch (pos) {
	 case kFender:
		 m_DesiredFlywheelSpeed = m_FlywheelSpeedFender;
		 break;
	 case kKeyNear:
		 m_DesiredFlywheelSpeed = m_FlywheelSpeedNearKey;
		 break;
	 case kKeyFar:
		 m_DesiredFlywheelSpeed = m_FlywheelSpeedFarKey;
		 break;
	 default:
		 break;
	 }
 }

 void ShooterController::RequestInfiniteConveyor() {
	m_bRequestInfiniteConveyor = true;
	m_bConveyorDone = false;
	m_InitialConveyorEncoderValue = robot->GetConveyorEncoderValue();
 }
 
 void ShooterController::RequestInfiniteConveyorOff(){
	 m_bRequestInfiniteConveyor = false;
	 GoToIdle();
 }
 
 void ShooterController::RequestFlywheel(bool on) {
	 m_bFlywheelDesiredOn = on;
 }
 
 void ShooterController::Reset() {
	 mustReset = true;
 }
 
 void ShooterController::GoToIdle() {
	 mustGoToIdle = true;
 }

 void ShooterController::RefreshIni(){
	 m_FlywheelPID_P = (robot->pini)->getf("SHOOTER", "FlywheelPFAC", 0.00701);
	 m_FlywheelPID_I = (robot->pini)->getf("SHOOTER", "FlywheelIFAC", 0.00002501);
	 m_FlywheelPID_D = (robot->pini)->getf("SHOOTER","FlywheelDFAC", 0.01501);
	 m_FlywheelSpeedFarKey= (robot->pini)->getf("SHOOTER","FlywheelSpeedFarKey", 1600.01);
	 m_FlywheelSpeedNearKey = (robot->pini)->getf("SHOOTER","FlywheelSpeedNearKey", 1475.01);
	 m_FlywheelSpeedFender= (robot->pini)->getf("SHOOTER","FlywheelSpeedBumper", 1300.01);
	advanceBallEncoderTicks = robot->pini->geti("CONVEYOR", "AdvanceBallEncoderTicks", 201);
	printf("Reading from INI AdvanceBallEncoderTicks: %d\n", advanceBallEncoderTicks);
	printf("Reading from INI FlywheelPFAC = %f Flywheel IFAC = %f FlywheelDFAC = %f\n", m_FlywheelPID_P, m_FlywheelPID_I,  m_FlywheelPID_D);
	printf("Reading from INI file for Speed... bumper: %f near key: %f far key: %f\n", m_FlywheelSpeedFender, m_FlywheelSpeedNearKey, m_FlywheelSpeedFarKey);

 }
 double ShooterController::GetFlywheelSpeed(){ // time and distance as vars?
	 m_curr_time = robot->timer->Get();
	 double delta = ( m_curr_time - m_last_time );
	 double encoder = robot->shooterEncoder->Get();
	 double speed = (encoder - m_last_value) / (32.0*delta) ;
	 double rate = robot->shooterEncoder->GetRate();
	 // DO_PERIODIC(200, printf("** Delta Time: %f Curr: %f Last: %f Speed = %f Rate: %f\n", delta, (encoder), m_last_value, speed, rate));
	 m_last_time = m_curr_time ;
	 m_last_value = encoder ;
	 // return speed ;
	return rate ;// encoder ticks/second
 }

 double ShooterController::FlywheelPID(double desired, double current, double *diffFlywheelError) {
 	
 	static float last_error = 0.0;
 	float PFac = m_FlywheelPID_P;
 	float IFac = m_FlywheelPID_I;
 	float DFac = m_FlywheelPID_D;
 	float NError = desired - current; // NError should be the new value and *diffFlywheelError is the old value

 	// more than N speed- saturate
 	// also means we start deceleration with N ticks to go
 	//if(NError > 250.0) NError = 250.0; // these values

 	double pterm = PFac * NError;
 	
 	// saturation of the terms
 	if (pterm > 0.0) {pterm = min(pterm, 1.0);} //
 	else {pterm = max(pterm, -1.0);}//

 	double difErr = (NError - last_error);
  	// DO_PERIODIC(250, printf("**difErr = NError - last_error = %f - %f = %f\n", NError, last_error, difErr));
 
 	m_Flywheel_sum_error += NError ; // accumulate error for integral term
	// saturation of the I term - dont bother if I constant is zero (from Ini file)
 	if(IFac != 0.0) {
		m_Flywheel_sum_error = (m_Flywheel_sum_error > (1.0/IFac)) ? (1.0/IFac) : m_Flywheel_sum_error ;
		m_Flywheel_sum_error = (m_Flywheel_sum_error < (-1.0/IFac)) ? (-1.0/IFac) : m_Flywheel_sum_error ;
 	}
 	
 	double ierror = m_Flywheel_sum_error;
 	double iterm = IFac * ierror ;

 
 	
 	if(difErr >= fabs(NError)) difErr = 0.0 ; // account for first time


 	double dterm = DFac * (difErr);
 	double motorValue = pterm + iterm + dterm;//??? is the motor value supposed to be the encoder value. 

 	

 	// values are solely for testing purposes
 	// DO_PERIODIC(50, printf("** pterm = PFac * NError = %f * %f = %f\n", PFac, NError, pterm));
 	// DO_PERIODIC(50, printf("** dterm = DFac * difErr = %f * %f = %f\n", DFac, difErr, dterm));
 	// DO_PERIODIC(20, printf("** Err: %f iErr: %f DifErr: %f \n", NError, ierror, difErr));
 	// DO_PERIODIC(20, printf("** motorValue %f =  %f + %f + %f \n", motorValue, pterm, iterm, dterm));
 	// DO_PERIODIC(20, printf("** Err: %f Motor: %f\n", NError, motorValue));
 	
 	// saturation of the terms
 	if (motorValue > 0.0) {motorValue = min(motorValue, 1.0);} //
 	else {motorValue = max(motorValue, -1.0);}//

 	last_error = NError;
 	(*diffFlywheelError) = difErr;

 	return motorValue;
 }
 

		
ShooterController::~ShooterController()
{
}
