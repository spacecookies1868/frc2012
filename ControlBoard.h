#ifndef CONTROLBOARD_H_
#define CONTROLBOARD_H_

#include "WPILib.h"
#include "RemoteControl.h"
#include "RobotModel.h"
#include "ButtonReader.h"

class ToggleReader {
public:
	ToggleReader(Joystick *myJoy, int upButton, int downButton) : joy(myJoy), upB(upButton), downB(downButton) {};
	int GetToggleState() {
		if (joy->GetRawButton(upB)) return 1;
		if (joy->GetRawButton(downB)) return -1;
		return 0;
	}
private:
	Joystick *joy;
	int upB;
	int downB;
};

class ControlBoard : public RemoteController
{
public:
	ControlBoard(RobotModel* myRobot);
	virtual ~ControlBoard();

	virtual double GetLeftWheelDesiredSpeed() { return wheelSpeedSet.left; };

	virtual double GetRightWheelDesiredSpeed(){ return wheelSpeedSet.right; };
	
	virtual double GetShooterDesiredSpeed() { return desiredShooterSpeed; };

	virtual bool GearShiftDesired() { return gearShiftDesired; };
	
	virtual bool LowGearDesired() { return lowGearDesired; };

	virtual bool CameraImageDesired() { return cameraImageDesired; };

	virtual bool PopUpDesired() { return popUpDesired; };
	
	virtual bool DemoAutoDriveDesired() { return demoAutoDriveDesired; };
	
	virtual bool ToggleConveyorDesired() { return toggleConveyorDesired; };
	
	virtual bool JogConveyorDesired() { return jogConveyorDesired; };
	
	virtual bool ReverseConveyorDesired() { return reverseConveyorDesired; };
	
	virtual bool PickUpToggleDesired(){ return pickUpDesired; };
	
	virtual bool ReversePickUpToggleDesired() { return reversePickUpDesired; }; //*
	
	virtual bool ShootingDesired() { return shootingDesired; }; // this will always return false
	
	virtual bool RampManipDesired() {return rampManipDesired; };
	
	virtual bool RampManipDownDesired() { return rampManipDownDesired; };
	
	virtual bool RampLegsDesired() { return rampLegsDesired; };
	
	virtual bool RampLegsDownDesired() { return rampLegsDownDesired; };
	
	virtual bool BrakeToggleDesired() { return brakeToggleDesired; };
	
	virtual bool HoodAdjustDesired() { return hoodAdjustDesired; };
		
	virtual bool TargetPivotDesired() { return targetPivotDesired; };
	
	virtual bool IssDriveForwardDesired() { return issDriveForwardDesired; };
	
	virtual bool IssAimDesired() { return issAimDesired; };
	
	virtual bool IssShootDesired() { return issShootDesired; };
	
	virtual bool DemoAutoPivotDesired() { return demoAutoPivotDesired; };
	
	virtual int ShootingPosition() { return shootingPosition; };
	
	virtual double AutoDelayValue() { return autoDelayValue; };
	
	virtual double GetFlywheelSpeedAdjustMultiplier() {
		return (1.0 + (flywheelSpeedAdjustVal / 10.0));
	}
	
	virtual bool BrakeDesiredOn() { return brakeDesiredOn; };
	
	virtual bool FlywheelDesiredOff() { return flywheelDesiredOff; };
	
	virtual bool ToggleReverseRollersDesired() { return toggleReverseRollersDesired; }
	
	virtual bool AutoTipBridgeDesired() { return autoTipBridgeDesired; }
	
	virtual bool AdvanceBallDesired() { return advanceBallDesired; }
	
	virtual bool ShootFarKeyDesired() { return shootFarKeyDesired; }
	
	virtual int ConveyorToggleState() { return conveyorToggleState; }
	
	virtual bool IntakeToggleDesired() { return intakeToggleDesired; }
	
	virtual void ReadControls();
	
	

private:
	bool tankdrive;
	Joystick* leftJoy;
	Joystick* rightJoy;
	Joystick* operatorJoy;
	// Joystick* ISSJoy;
	RobotModel* robot;
	
	ButtonReader* gearShiftButton;
	ButtonReader* reverseDriveButton;
	ButtonReader* cameraButton;
	ButtonReader* autoDriveButton;
	ButtonReader* popUpButton;
	ButtonReader* rampManipButton;
	ButtonReader* rampLegsButton;
	
	ButtonReader* toggleConveyorButton;
	ButtonReader* reverseConveyorButton;
	ButtonReader* jogConveyorButton;
	ButtonReader* pickUpButton;
	ButtonReader* reversePickUpButton;//*
	ButtonReader* intakeMechanismButton;
	ButtonReader* brakeSwitch;
	ButtonReader* hoodAdjustButton;
	
	ButtonReader* issDriveForwardButton;
	ButtonReader* issAimButton;
	ButtonReader* issShootButton;
	
	ButtonReader* targetPivotButton;
	ButtonReader* cameraButton2;
	
	ButtonReader* testAutoDriveButton;
	ButtonReader* testAutoPivotButton;
	
	ButtonReader* quickTurnButton;
	
	ButtonReader* shootPosFenderButton;
	ButtonReader* shootPosKeyNearButton;
	ButtonReader* shootPosKeyFarButton;
	ToggleReader* conveyorToggleReader;
	ButtonReader* reverseRollersButton;
	ButtonReader* advanceBallButton;
	
	
	double desiredLeftSpeed;
	double desiredRightSpeed;
	double desiredShooterSpeed ;

	bool isQuickTurn;
	bool gearShiftDesired;
	bool lowGearDesired;
	bool lowGearDesiredLast;
	bool reverseDriveDesired;
	bool cameraImageDesired;
	bool popUpDesired;
	bool rampManipDesired;
	bool rampManipDownDesired;
	bool rampManipDownDesiredLast;  
	bool demoAutoDriveDesired;
	bool jogConveyorDesired;
	bool toggleConveyorDesired;
	bool reverseConveyorDesired;
	bool pickUpDesired;
	bool reversePickUpDesired;
	bool shootingDesired;
	bool brakeToggleDesired;
	bool hoodAdjustDesired;
	bool issDriveForwardDesired;
	bool issAimDesired;
	bool issShootDesired;
	bool targetPivotDesired;
	bool demoAutoPivotDesired;
	bool popUpButtonState;
	bool popUpButtonStateLast;
	bool brakeSwitchPosLast;
	bool brakeDesiredOn;
	bool flywheelDesiredOff;
	bool toggleReverseRollersDesired;
	bool autoTipBridgeDesired;
	bool advanceBallDesired;
	bool shootFarKeyDesired;
	bool intakeToggleDesired;
	bool rampLegsDesired;
	bool rampLegsDownDesired;
	
	int shootingPosition;
	double autoDelayValue;
	double autoSwitchVal;
	
	int conveyorToggleStateLast;
	int conveyorToggleState;
	double flywheelSpeedAdjustVal;
	
	virtual MotorSpeedSet CalculateTankDriveMotorValues(double joy1, double joy2);

	virtual MotorSpeedSet CalculateArcadeDriveMotorValues(double joy1, double joy2);
};

#endif /*CONTROLBOARD_H_*/
