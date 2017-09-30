#ifndef SHOOTERCONTROLLER_H_
#define SHOOTERCONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"

#define IP_FILTER_SIZE 3
#define OP_FILTER_SIZE 3

class ShooterController
{
public:
	ShooterController(RobotModel*, RemoteController*);
	void Update(double currTimeSec, double deltaTimeSec);
	virtual ~ShooterController();
	void SetShooterSpeed(double speed);
	void SetShooterSpeedByPosition(int pos);
	void RequestInfiniteConveyor();
	void RequestInfiniteConveyorOff();
	void RequestFlywheel(bool on);
	void RefreshIni();
	double GetFlywheelSpeed();
	void Reset();
	void GoToIdle();
	double FlywheelPID(double m_DesiredFlywheelSpeed, double m_CurrentFlywheelSpeed, double *diffFlywheelError); // Speed in encoder ticks per second
	double InputFilter(double value);
	double OutputFilter(double value);
	void ResetFilters();
	
	enum ShooterState {
			//states
			kReset, kInitialize, kIdle, kAdvanceConveyorFinite,kAdvanceConveyorInfinite, kReverseConveyor, kReverseConveyorDone
		
		};

private:
	RobotModel *robot;
	RemoteController *humanControl;
	
	bool m_bTogglePickup;
	bool m_bToggleReversePickup; //*
	bool m_bToggleConveyor;
	bool m_bRequestConveyor;
	bool m_bRequestShooter;
	bool m_bFlywheelDesiredOn;
	bool m_bAdvanceBallRequested;

	bool m_bBallIntakeSensed;
	bool m_bPreviousBallIntakeSensed;
	bool m_bNeedToMoveUpSingleBall;
	
	bool m_bRequestReverseConveyor;

	bool m_bRollerMotorsOnReverse; 
	bool m_bRollerMotorsOn;
	bool m_bConveyorDone;
	bool m_bReverseConveyorDone;
		
	double m_FlyWheelSpeed;
	double m_RollerSpeed;
	
	double m_ConveyorEncoderValue;
	double m_InitialConveyorEncoderValue;
	double m_DiffConveyorEncoderValue;
	
	double m_InitialFlywheelTime;
	double m_InitialFlywheelEncoderVal;
	double m_FlywheelStallTime;
	double m_MinFlywheelEncoderChange;
	
	double m_CurrentFlywheelEncoderVal;
	double m_CurrentFlywheelSpeed;
	double m_GivenFlywheelSpeed;
	double m_DiffFlywheelSpeed;
	
	double m_DefaultConveyorSpeed;
	double advanceBallStopRollerTimerBegin;
	double advanceBallConveyorStartTimerBegin;
	int advanceBallEncoderTicks;
	int m_ConveyorToggleState;
	
	bool m_bRequestJogConveyor;
	bool m_bSensorRequestedAdvance;
	bool m_bRequestToggleConveyor;
	bool m_bAdvanceBall;
			
	bool m_bRequestInfiniteConveyor;
	
	bool m_bRequestHoodAdjust;
	
	double m_ConveyorGoalDistance;
	
	double m_curr_time, m_last_time, m_last_value ;
	double m_FlywheelMotorVal, m_Flywheel_sum_error;
	double m_DesiredFlywheelSpeed;
	double m_FlywheelSpeedNearKey;
	double m_FlywheelSpeedFarKey;
	double m_FlywheelSpeedFender;
	double m_FlywheelPID_P, m_FlywheelPID_I, m_FlywheelPID_D;
	float m_ifilter[IP_FILTER_SIZE], m_ofilter[OP_FILTER_SIZE];
	int m_ifilterindex, m_ofilterindex ;
	
	bool mustReset;
	bool mustGoToIdle;

	int m_LastShootingPosition;
	int m_ShootingPosition;
	bool m_bShootingPositionChanged;
	double m_FlywheelSpeedAdjustMultiplier;
	
	bool m_bHumanRequestsFlywheelOff;
	bool m_bNeedToTurnOffRollers;
	
	bool m_bIntakeToggleDesired;
	
	uint32_t m_stateVal;
	uint32_t nextState;
};

#endif /*SHOOTERCONTROLLER_H_*/
