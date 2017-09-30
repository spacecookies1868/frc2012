#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2012.h"
#include "Debugging.h"
#include <math.h>
#define NEW_CONTROL_BOARD 1

ControlBoard::ControlBoard(RobotModel* myRobot){
	leftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);
	// ISSJoy = operatorJoy ;
	// ISSJoy = new Joystick(ISS_JOY_USB_PORT);
	
	tankdrive = false;
	wheelSpeedSet.left = 0;
	wheelSpeedSet.right = 0;
	desiredShooterSpeed = 0;
 
	robot = myRobot;

	gearShiftDesired = false;
	lowGearDesired = false;
	lowGearDesiredLast = false;
	cameraImageDesired = false;
	demoAutoDriveDesired = false;
	popUpDesired = false;
	rampManipDesired = false;
	rampManipDownDesired = false;
	//jogConveyorDesired = false;
	toggleConveyorDesired = false;
	reverseConveyorDesired = false;
	pickUpDesired = false;
	reversePickUpDesired = false;//*
	shootingDesired = false;
	brakeToggleDesired = false;
	hoodAdjustDesired = false;//
	issDriveForwardDesired = false;
	issAimDesired = false;
	issShootDesired = false;
	reverseDriveDesired = false;
	popUpButtonState = false;
	popUpButtonStateLast = false;
	brakeSwitchPosLast = false;
	brakeDesiredOn = false;
	flywheelDesiredOff = false;
	toggleReverseRollersDesired = false;
	autoTipBridgeDesired = false;
	advanceBallDesired = false;
	shootFarKeyDesired = false;
	rampLegsDesired = false;
	rampLegsDownDesired = false;
	
	conveyorToggleStateLast = 0;
	conveyorToggleState = 0;
	flywheelSpeedAdjustVal = 0;
	autoSwitchVal = 0;
	
	shootingPosition = kDefaultPosition;
		
	autoDriveButton = new ButtonReader(rightJoy, 4); // doesn't exist
	gearShiftButton = new ButtonReader(rightJoy, 3); // 
	
	popUpButton = new ButtonReader(leftJoy, 3);
	rampManipButton = new ButtonReader(leftJoy, 1);
	rampLegsButton = new ButtonReader(rightJoy, 2);
	reverseDriveButton = new ButtonReader(rightJoy, 8);

	brakeSwitch = new ButtonReader(leftJoy, 6);
	
	issDriveForwardButton = new ButtonReader(rightJoy, 6);
	issAimButton = new ButtonReader(rightJoy, 7);
	issShootButton = new ButtonReader(rightJoy, 8);
	
	testAutoDriveButton = new ButtonReader(rightJoy, 10);
	testAutoPivotButton = new ButtonReader(leftJoy, 4);
	
	quickTurnButton = new ButtonReader(rightJoy, 1);
	

	
#ifdef NEW_CONTROL_BOARD
	shootPosKeyFarButton = new ButtonReader(operatorJoy, 1);
	reverseRollersButton = new ButtonReader(operatorJoy, 9);//put this futher away
	conveyorToggleReader = new ToggleReader(operatorJoy, 3, 4);
	pickUpButton = new ButtonReader(operatorJoy, 5);
	shootPosFenderButton = new ButtonReader(operatorJoy, 6);
	shootPosKeyNearButton = new ButtonReader(operatorJoy, 7);
	intakeMechanismButton = new ButtonReader(operatorJoy, 2);// moved this closer, where reverse roller was
	advanceBallButton = new ButtonReader(operatorJoy, 8);//swap 8&9 so adv.BB closer to pickupB
	brakeSwitch = new ButtonReader(operatorJoy, 10);
	//driveJogForwardButton = new ButtonReader(operatorJoy, 11);
	//driveJogBackwardsButton = new ButtonReader(operatorJoy, 12);
	
	// dummy buttons
	cameraButton = new ButtonReader(leftJoy, 12); 
	hoodAdjustButton = new ButtonReader(leftJoy, 12);
	jogConveyorButton = new ButtonReader(leftJoy, 12);
	reverseConveyorButton = new ButtonReader(leftJoy, 12);
	reversePickUpButton = new ButtonReader(leftJoy, 12);
	targetPivotButton = new ButtonReader(leftJoy, 12);
	cameraButton2 = new ButtonReader(leftJoy, 12);
	toggleConveyorButton = new ButtonReader(leftJoy, 12);

	
#else
	cameraButton = new ButtonReader(operatorJoy, 2);
	hoodAdjustButton = new ButtonReader(operatorJoy, 1);//**
	jogConveyorButton = new ButtonReader(operatorJoy, 3);
	reverseConveyorButton = new ButtonReader(operatorJoy, 8);
	pickUpButton = new ButtonReader(operatorJoy, 4); // operatorJoy 6
	reversePickUpButton = new ButtonReader(operatorJoy, 7);//*
	intakeMechanismButton = new ButtonReader(operatorJoy, 5); // operatorJoy 9
	targetPivotButton = new ButtonReader(operatorJoy, 6);
	cameraButton2 = new ButtonReader(operatorJoy, 6); // MUST be the same as the target pivot button

	shootPosFenderButton = new ButtonReader(operatorJoy, 5); 
	shootPosKeyNearButton = new ButtonReader(operatorJoy, 12); // this button doesn't exist
	shootPosKeyFarButton = new ButtonReader(operatorJoy, 12);
	
#endif
	
}

/**
 * takes the Y axis values of the right and left joysticks and sets the motorSpeed for the left and right
 * motors to the corresponding raw joystick value
 */
MotorSpeedSet ControlBoard::CalculateTankDriveMotorValues(double leftJoyY, double rightJoyY) {

	if(reverseDriveDesired) {
			leftJoyY = -leftJoyY;
	}
	
	// Note: Reversing one side (the left side) is handled in RobotModel::SetWheelSpeed, so for now,
	// we're going to work as if both sides have the same sign.

	wheelSpeedSet.left = -leftJoyY;
	wheelSpeedSet.right = -rightJoyY;
	return wheelSpeedSet;
}

/**
 * Takes the raw inputs from the joysticks (the Y axis on one joystick and the X axis on the other)
 * and calculates the values to be fed to the left and right motors
 * 
 * @param speedJoyY the value from the Y axis of the "throttle" or "speed" Joystick (left joystick)
 * @param dirJoyX the value from the X axis of the "wheel" or "direction" Joystick (right joystick)
 */
MotorSpeedSet ControlBoard::CalculateArcadeDriveMotorValues(double speedJoyY, double dirJoyX) {
	//printf("In Arcade code now \n");
	dirJoyX = -dirJoyX;
	if(reverseDriveDesired) {
		speedJoyY = -speedJoyY;
	}
	//isQuickTurn = (rightJoy->GetButton(Joystick::kTriggerButton));
	bool isHighGear = !(robot->IsLowGear()); 

	double wheel = dirJoyX;

	if(fabs(wheel) < 0.1) //If user does not want to steer, stop and cancel acceleration
		wheel = 0.0;

	static double old_wheel = 0.0;
	double neg_inertia = wheel - old_wheel;
	old_wheel = wheel;

	double left_pwm, right_pwm, overPower;
	float sensitivity = 1.7;

	float angular_power;
	float linear_power;

	static int i = 0;
	i++;

#define M_PI 3.1415926535

	// 2012 version - added wheel nonlinearity - to affect steering feel at low speeds
	double wheelNonLinearity;
	if (isHighGear) {
		wheelNonLinearity = 0.7; // used to be csvReader->TURN_NONLIN_HIGH
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
	} else {
		wheelNonLinearity = 0.4; // used to be csvReader->TURN_NONLIN_LOW
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
	}
	
	static double neg_inertia_accumulator = 0.0;
	double neg_inertia_scalar;
	if (isHighGear) {
		// printf("High ");
		neg_inertia_scalar = 20.0; // used to be csvReader->NEG_INTERTIA_HIGH
		sensitivity = 1.22; // used to be csvReader->SENSE_HIGH
	} else {
		// printf("Low ");
		if (wheel * neg_inertia > 0) {
			neg_inertia_scalar = 10.0; // used to be csvReader->NEG_INERTIA_LOW_MORE
		} else {
			if (fabs(wheel) > 0.65) {
				neg_inertia_scalar = 10.0;// used to be csvReader->NEG_INTERTIA_LOW_LESS_EXT
			} else {
				neg_inertia_scalar = 5.0; // used to be csvReader->NEG_INTERTIA_LOW_LESS
			}
		}
		sensitivity = 1.24; // used to be csvReader->SENSE_LOW

		if (fabs(speedJoyY) > 0.1) { // used to be csvReader->SENSE_CUTTOFF
			sensitivity = 1 - (1 - sensitivity) / fabs(speedJoyY);
		}
	}
	double neg_inertia_power=neg_inertia * neg_inertia_scalar;
	neg_inertia_accumulator+=neg_inertia_power;
	// printf("neg_inertia_power=%f accumulator=%f\n", neg_inertia_power, neg_inertia_accumulator);

	wheel = wheel + neg_inertia_accumulator;
	if(neg_inertia_accumulator > 1)
		neg_inertia_accumulator -= 1;
	else if (neg_inertia_accumulator < -1)
		neg_inertia_accumulator += 1;
	else
		neg_inertia_accumulator = 0;

	linear_power = speedJoyY;

	if (isQuickTurn) {
		overPower = 1.0;
		if (isHighGear) {
			sensitivity = 1.0;
		} else {
			sensitivity = 1.0;
		}
		angular_power = wheel;
	} else {
		overPower = 0.0;
		angular_power = fabs(speedJoyY) * wheel * sensitivity;
	}

	right_pwm = left_pwm = linear_power;
	left_pwm += angular_power;
	right_pwm -= angular_power;

	if (left_pwm > 1.0) {
		right_pwm -= overPower*(left_pwm - 1.0);
		left_pwm = 1.0;
	} else if (right_pwm > 1.0) {
		left_pwm -= overPower*(right_pwm - 1.0);
		right_pwm = 1.0;
	} else if (left_pwm < -1.0) {
		right_pwm += overPower*(-1.0 - left_pwm);
		left_pwm = -1.0;
	} else if (right_pwm < -1.0) {
		left_pwm += overPower*(-1.0 - right_pwm);
		right_pwm = -1.0;
	}

	// if (i % 50 == 0) {
		// 	printf("l: %f r: %f t: %f w: %f ax: %f\n", left_pwm, right_pwm, throttle, wheel, axis);
	// }

	// Note: Reversing one side (the left side) is handled in RobotModel::SetWheelSpeed, so for now,
	// we're going to work as if both sides have the same sign.
	wheelSpeedSet.left = left_pwm ;
	wheelSpeedSet.right = right_pwm ;

	return wheelSpeedSet;
}

/**
 * Reads and inteprets human inputs
 */
void ControlBoard::ReadControls() {
	//read shooter joy values here
	//desiredShooterSpeed = operatorJoy->GetRawAxis(3) ; // read z wheel for shooter speed edit: will not use joystick
	
	if (tankdrive) {
		wheelSpeedSet = 
				CalculateTankDriveMotorValues(leftJoy->GetRawAxis(2), rightJoy->GetRawAxis(2));
		// Left Joystick Y and Right Joystick Y
	} 
	else { // Arcade Drive
		wheelSpeedSet = 
				CalculateArcadeDriveMotorValues(leftJoy->GetRawAxis(2), rightJoy->GetRawAxis(1));
		// Assuming speed controlled by left joystick and direction controlled by right.
		// May be the wrong assumption -- should be made settable.
	}
	
    autoDelayValue = (( 1 + operatorJoy->GetRawAxis(3) ) * 10);

	if (shootPosFenderButton->WasJustPressed()) {
		if (robot->IsFlywheelOn()) {
			flywheelDesiredOff = true;
			shootingPosition = kDefaultPosition;
		}
		else {
			shootingPosition = kFender;
			flywheelDesiredOff = false;
		}
	}
	else if (shootPosKeyNearButton->WasJustPressed()) {
		if (robot->IsFlywheelOn()) {
			flywheelDesiredOff = true;
			shootingPosition = kDefaultPosition;
		} else {
			shootingPosition = kKeyNear;
			flywheelDesiredOff = false;
		}
	}
	else if (shootPosKeyFarButton->WasJustPressed()) {
		if (robot->IsFlywheelOn()) {
			flywheelDesiredOff = true;
			shootingPosition = kDefaultPosition;
		} else {
			shootingPosition = kKeyFar;
			flywheelDesiredOff = false;
		}
	}
	else flywheelDesiredOff = false;
	
	
	lowGearDesiredLast = lowGearDesired;
	lowGearDesired = gearShiftButton->IsDown();
	gearShiftDesired = (lowGearDesiredLast != lowGearDesired);
	
	/*rampManipDownDesiredLast = rampManipDownDesired;
	rampManipDownDesired = rampManipButton->IsDown();
	rampManipDesired = (rampManipDownDesiredLast != rampManipDownDesired);*/
	
	//popUpButtonStateLast = popUpButtonState;
	//popUpButtonState = popUpButton->IsDown();
	//popUpDesired = (popUpButtonStateLast != popUpButtonState);
	popUpDesired = popUpButton->StateJustChanged(); // uncomment the prev 3 lines and remove this one 
													// if popup doesn't work
	
	if ( reverseDriveButton->WasJustPressed() ) reverseDriveDesired = !reverseDriveDesired;
	
	
	cameraImageDesired = ( cameraButton->WasJustPressed() || cameraButton2->WasJustPressed() );
	if (cameraImageDesired) printf("pressing camera button\n");

	//popUpDesired = popUpButton->WasJustPressed();
	demoAutoDriveDesired = ( autoDriveButton->WasJustPressed() || testAutoDriveButton->WasJustPressed() );
	// jogConveyorDesired = jogConveyorButton->WasJustPressed(); // OLD CONTROL BOARD
	// toggleConveyorDesired = toggleConveyorButton->WasJustPressed(); // OLD CONTROL BOARD
	// reverseConveyorDesired = reverseConveyorButton->WasJustPressed(); // OLD CONTROL BOARD
	pickUpDesired = pickUpButton->WasJustPressed();
	reversePickUpDesired = reversePickUpButton->WasJustPressed(); //* 
	intakeToggleDesired = intakeMechanismButton->WasJustPressed();
	
	rampManipDesired = rampManipButton->WasJustPressed();
	if (rampManipDesired) 	rampManipDownDesired = !rampManipDownDesired;
	
	rampLegsDesired = rampLegsButton->WasJustPressed();
	if (rampLegsDesired)	rampLegsDownDesired = !rampLegsDownDesired;
	
	hoodAdjustDesired = hoodAdjustButton->WasJustPressed();
	targetPivotDesired = targetPivotButton->WasJustPressed();
	demoAutoPivotDesired = testAutoPivotButton->WasJustPressed();
	advanceBallDesired = advanceBallButton->WasJustPressed(); 
	
	toggleReverseRollersDesired = reverseRollersButton->WasJustPressed();
	
	issDriveForwardDesired = issDriveForwardButton->WasJustPressed();
	issAimDesired = issAimButton->WasJustPressed();
	issShootDesired = issShootButton->WasJustPressed();
	
	if (operatorJoy->GetRawAxis(1) > 0.0) {
		shootFarKeyDesired = false;
	} else {
		shootFarKeyDesired = true;
	}
	
	if (operatorJoy->GetRawAxis(2) > 0.0) {
		autoTipBridgeDesired = true;
	} else {
		autoTipBridgeDesired = false;
	}
	
	if (shootingDesired && robot->IsFlywheelOn()) shootingPosition = kDefaultPosition;
	
	//brakeSwitchPosLast = brakeSwitchPos;
	//brakeSwitchPos = !(brakeSwitch->IsDown()); 
	//if (brakeSwitchPosLast != brakeSwitchPos) brakeToggleDesired = true;
	//else brakeToggleDesired = false;
	
	brakeToggleDesired = brakeSwitch->StateJustChanged();
	brakeDesiredOn = !(brakeSwitch->IsDown());	// inverted b/c brake switch is wired backwards
	isQuickTurn = quickTurnButton->IsDown();
	//rampManipDownDesired = rampManipButton->IsDown();
	
#ifdef NEW_CONTROL_BOARD
	
	conveyorToggleStateLast = conveyorToggleState;
	conveyorToggleState = conveyorToggleReader->GetToggleState();
	
	/*if (conveyorToggleStateLast == 0 && conveyorToggleState > 0) {
		// toggle was just pushed up
	} else if (conveyorToggleStateLast == 0 && conveyorToggleState < 0) {
		// toggle was just pushed down
	} else if (conveyorToggleStateLast > 0 && conveyorToggleState == 0) {
		// toggle was just released from being up
	} else if (conveyorToggleStateLast < 0 && conveyorToggleState == 0) {
		// toggle was just released from being down
	} else if (conveyorToggleStateLast > 0 && conveyorToggleState < 0) {
		// toggle somehow went instantaneously
	}*/

	
	if (conveyorToggleStateLast != conveyorToggleState) {
		
		if (conveyorToggleState > 0) {
			toggleConveyorDesired = true;
			reverseConveyorDesired = false;
			printf("conveyor forward\n");
		} else if (conveyorToggleState < 0) {
			reverseConveyorDesired = true;
			toggleConveyorDesired = false;
			printf("conveyor reverse\n");
		} else {
			// reverseConveyorDesired = false;
			if (conveyorToggleStateLast > 0) toggleConveyorDesired = true;
			else toggleConveyorDesired = false;
			if (conveyorToggleStateLast < 0) reverseConveyorDesired = true;
			else reverseConveyorDesired = false;
		}
	} else {
		toggleConveyorDesired = false;
		reverseConveyorDesired = false; 	
	}
	
	flywheelSpeedAdjustVal = -(operatorJoy->GetRawAxis(4));
	
	autoSwitchVal = operatorJoy->GetRawAxis(2);
	// DO_PERIODIC(500, printf("auto switch val is %f\n", autoSwitchVal));
	
#endif
	

}



ControlBoard::~ControlBoard()
{
}

