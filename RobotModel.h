#include "Debugging.h"
#include "WPILib.h"
#include "LinearVictor.h"
#include "ini.h"
#include "Debugging.h"

#ifndef ROBOTMODEL_H_
#define ROBOTMODEL_H_

class RobotModel
{
public:
	
	enum Wheels { kLeftWheel, kRightWheel, kBothWheels };
	
	Ini * pini ;
	LinearVictor* driveVictorLeftA;
	LinearVictor* driveVictorLeftB;
    LinearVictor* driveVictorLeftC;
	LinearVictor* driveVictorRightA;
	LinearVictor* driveVictorRightB;
    LinearVictor* driveVictorRightC;
	LinearVictor* flywheelVictorA;
	LinearVictor* flywheelVictorB;
	LinearVictor* rollerVictor;
	LinearVictor* conveyorVictor;
	
	Encoder* rightWheelEncoder;
	Encoder* leftWheelEncoder;
	Encoder* shooterEncoder;
	Encoder* conveyorEncoder;
	
	DriverStationLCD* dsLCD;
	
	Compressor* compressor;
	ADXL345_I2C* accel;
	DigitalInput* popUpBeamSensor;
	DigitalInput* intakeSensor;
	
	AxisCamera* camera;
	
	Gyro* gyro;
	
	Timer* timer;
	
	bool isLowGear;
	bool rampArmIsUp;
	bool rampLegsAreUp;
	bool popUpEngaged;
	bool hoodLowered;
	//bool useCamera;
	bool brakeIsOn;
	bool hasCamera ;
	bool isFlywheelOn;
	bool useDebugPrintfs;
	
	RobotModel();
	bool BeamSensorActivated();
	bool IntakeSensorActivated();
	void EnableCompressor();
	void DisableCompressor();
	bool GetCompressorState();
	bool CheckCameraConnection();
	
	void PrintDebugInfoToLCD();
	
	void ResetGyro();
	float GetGyroAngle();
	
//	void PrintToLCD(int line, char message);
	double GetWheelEncoderValue(RobotModel::Wheels w);
	double GetWheelEncoderDistance(RobotModel::Wheels w);
	double GetRightWheelEncoderValue();
	double GetLeftWheelEncoderValue();
	double GetShooterEncoderValue();
	double GetConveyorEncoderValue();
	
	void ResetWheelEncoder(RobotModel::Wheels w = RobotModel::kBothWheels);
	void ResetShooterEncoder();
	void ResetConveyorEncoder();
	
	void SetWheelSpeed(double speed, RobotModel::Wheels w);
	void SetFlywheelSpeed (double speed);
	void SetRollerSpeed (double speed);
	void SetConveyorSpeed (double speed);
	void SetBrakeOn();
	void SetBrakeOff();
	void ToggleBrake();
	bool BrakeIsOn();
	
	bool IsLowGear();
	void ShiftToHighGear();
	void ShiftToLowGear();
	void ShiftGear();
	bool RampArmIsUp();
	bool RampLegsAreUp();
	void ManipRamp();
	void RaiseRampLegs();
	void LowerRampLegs();
	void ToggleIntake();
	void RaiseIntake();
	void LowerIntake();
	void ToggleHood();
	void LowerHood();
	void RaiseHood();
	bool IsHoodLowered();
	void TogglePopUp();
	void EngagePopUp();
	void DisengagePopUp();
	double GetCurrentTimeInSeconds();
	
#ifdef USE_CAMERA
	HSLImage* GetCameraImage();
#endif
	
	bool HasCamera() { return hasCamera;} ;
	bool IsFlywheelOn() { return isFlywheelOn; };
	
	virtual ~RobotModel();	
	
private:
	
	Solenoid* shifterSolenoid;
	Solenoid* popUpSolenoid;
	Solenoid* rampArmUpSolenoid;
	Solenoid* rampArmDownSolenoid;
	Solenoid* rampLegsSolenoid;
	Solenoid* brakeOnSolenoid;
	Solenoid* brakeOffSolenoid;
	Solenoid* hoodSolenoid;
};

#endif /*ROBOTMODEL_H_*/
