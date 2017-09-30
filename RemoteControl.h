#ifndef REMOTECONTROLLER_H_
#define REMOTECONTROLLER_H_

typedef struct {
	double left;
	double right;
} MotorSpeedSet;

enum ShootingPosition {
	kFender, kKeyNear, kKeyFar, kDefaultPosition
};


class RemoteController {
	
public:
	
    //What the User wants
	/**
	 * Returns the desired speed of the left motor.
	 *
	 * @return the desired speed of the left motor
	 */
	//virtual MotorSpeedSet GetDesiredMotorSpeedSet() = 0;
	virtual double GetLeftWheelDesiredSpeed() = 0;
    
    /**
	 * Returns the desired speed of the right motor.
	 *
	 * @return the desired speed of the right motor
	 */
	virtual double GetRightWheelDesiredSpeed() = 0;

    /**
	 * Returns the desired speed of the shooter.
	 *
	 * @return the desired speed of the shooter
	 */
	virtual double GetShooterDesiredSpeed() = 0;

	//virtual int GetDesiredGear() = 0;
	
	virtual bool GearShiftDesired() = 0;
	
	virtual bool LowGearDesired() = 0;
	
	virtual bool RampManipDesired() = 0;
	
	virtual bool RampManipDownDesired() = 0;
	
	virtual bool RampLegsDesired() = 0;
	
	virtual bool RampLegsDownDesired() = 0;
	
	virtual bool CameraImageDesired() = 0;
	
	virtual bool PopUpDesired() = 0;
	
	virtual bool DemoAutoDriveDesired() = 0;
	
	virtual bool ToggleConveyorDesired() = 0;
	
	virtual bool JogConveyorDesired() = 0;
	
	virtual bool ReverseConveyorDesired() = 0;
	
	virtual bool PickUpToggleDesired() = 0;
	
	virtual bool AdvanceBallDesired() = 0;
	
	virtual bool ReversePickUpToggleDesired() = 0;
	
	virtual bool ShootingDesired() = 0;
	
	virtual bool BrakeToggleDesired() = 0;
	
	virtual bool HoodAdjustDesired() = 0;
		
	virtual bool TargetPivotDesired() = 0;
	
	virtual bool IssDriveForwardDesired() = 0;
	
	virtual bool IssAimDesired() = 0;
	
	virtual bool IssShootDesired() = 0;
	
	virtual bool DemoAutoPivotDesired() = 0;
	
	virtual int ShootingPosition() = 0;
	
	virtual double GetFlywheelSpeedAdjustMultiplier() = 0;
	
	virtual bool BrakeDesiredOn() = 0;
	
	virtual bool FlywheelDesiredOff() = 0;
	
	virtual bool ToggleReverseRollersDesired() = 0;
	
	virtual bool AutoTipBridgeDesired() = 0;
	
	virtual int ConveyorToggleState() = 0;
	
	virtual bool ShootFarKeyDesired() = 0;
	
	virtual bool IntakeToggleDesired() = 0;
	
    /**
	 * Updates the state of the controller
	 */
	virtual void ReadControls() = 0;
	
	virtual ~RemoteController() {}
	
protected:
	MotorSpeedSet wheelSpeedSet;
	
};

#endif
