#include "WPILib.h"
#include "LinearVictor.h"
#include "RobotModel.h"
#include "RobotPorts2012.h"
#include "Debugging.h"
#include <math.h>

RobotModel::RobotModel() {

	pini = new Ini("/robot.ini");
		
	driveVictorLeftA  = new LinearVictor(LEFT_MOTOR_A_PWM_PORT);
	driveVictorLeftB  = new LinearVictor(LEFT_MOTOR_B_PWM_PORT);
    driveVictorLeftC  = new LinearVictor(LEFT_MOTOR_C_PWM_PORT);
	
	driveVictorRightA = new LinearVictor(RIGHT_MOTOR_A_PWM_PORT);
	driveVictorRightB = new LinearVictor(RIGHT_MOTOR_B_PWM_PORT);
    driveVictorRightC = new LinearVictor(RIGHT_MOTOR_C_PWM_PORT);
	
	flywheelVictorA = new LinearVictor(FLYWHEEL_MOTOR_A_PWM_PORT);
	flywheelVictorB = new LinearVictor(FLYWHEEL_MOTOR_B_PWM_PORT);
	rollerVictor 	= new LinearVictor(ROLLER_MOTOR_PWM_PORT);
	conveyorVictor 	= new LinearVictor(CONVEYOR_MOTOR_PWM_PORT);

	popUpBeamSensor = new DigitalInput(BEAM_SENSOR_PORT);	
	intakeSensor = new DigitalInput(INTAKE_SENSOR_PORT);
	
	shifterSolenoid = new Solenoid(GEAR_SHIFT_SOLENOID_CHAN);
	popUpSolenoid = new Solenoid(POP_UP_SOLENOID_CHAN);
	
	rampArmUpSolenoid = new Solenoid(RAMP_ARM_UP_SOLENOID_CHAN);
	rampArmDownSolenoid = new Solenoid(RAMP_ARM_DOWN_SOLENOID_CHAN);
	
	rampLegsSolenoid = new Solenoid(RAMP_LEGS_SOLENOID_CHAN);
	
	brakeOnSolenoid = new Solenoid(BRAKE_ON_SOLENOID_CHAN);
	brakeOffSolenoid = new Solenoid(BRAKE_OFF_SOLENOID_CHAN);
	
	hoodSolenoid = new Solenoid(HOOD_ADJUST_SOLENOID_CHAN);
	
	leftWheelEncoder = new Encoder(LEFT_WHEEL_ENCODER_A_PWM_PORT, LEFT_WHEEL_ENCODER_B_PWM_PORT, true);
	rightWheelEncoder = new Encoder(RIGHT_WHEEL_ENCODER_A_PWM_PORT, RIGHT_WHEEL_ENCODER_B_PWM_PORT, true);
	
	shooterEncoder = new Encoder(FLYWHEEL_ENCODER_A_PWM_PORT, FLYWHEEL_ENCODER_B_PWM_PORT, true) ;
	conveyorEncoder = new Encoder(CONVEYOR_ENCODER_A_PWM_PORT, CONVEYOR_ENCODER_B_PWM_PORT, true);

	dsLCD = DriverStationLCD::GetInstance();
	
	popUpEngaged = false;
	hoodLowered = false;
	rampLegsAreUp = true;
	
	leftWheelEncoder->Start();
	rightWheelEncoder->Start();
	conveyorEncoder->Start();
	shooterEncoder->Start();
	
	timer = new Timer();
	timer->Start();
	
	isLowGear = false;
	brakeIsOn = false;
	hasCamera = false;
	
	useDebugPrintfs = pini->geti("DEBUG", "NearKeyForwardDist", 0);
	
#ifdef USE_CAMERA	
	
	hasCamera = CheckCameraConnection(); // returns true if camera is connected at 10.18.68.11, false if not.
										 // ... but right now it always returns false.
	if (hasCamera) {
		printf("!!! Camera Detected\n");
		camera = &(AxisCamera::GetInstance(/*"10.18.68.12"*/));
		camera->WriteResolution(AxisCamera::kResolution_640x480);
		camera->WriteCompression(20);
		camera->WriteBrightness(0);
	} else {
		camera = NULL;
	}
	
#endif
	
	compressor = new Compressor(COMPRESSOR_PRESSURE_SWITCH_CHAN, COMPRESSOR_RELAY_CHAN);	
	gyro = new Gyro(GYRO_PORT);
	gyro->Reset();
	gyro->SetSensitivity(.007);	// used to be .0033, last year used .007
	
}

bool RobotModel::CheckCameraConnection() {
	// add implementation here
	return false;
}

void RobotModel::EnableCompressor()  {
	compressor->Start();
}

void RobotModel::DisableCompressor()  {
	compressor->Stop();
}

bool RobotModel::GetCompressorState()  {
	return compressor->Enabled();
}

void RobotModel::PrintDebugInfoToLCD() {
	if (useDebugPrintfs) {
		dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Enc L %d R %d C %d       ", 
				(int)GetLeftWheelEncoderValue(), 
				(int)GetRightWheelEncoderValue(), 
				(int)GetConveyorEncoderValue()
				);			
	}
	
}

bool RobotModel::BeamSensorActivated(){
	//need the test the values
	//assumes that 0 = not over bump, 1 = over bump
	bool noObstruction = false;
	if (popUpBeamSensor->Get() == 0){
		return noObstruction;
	}
	else return !noObstruction;
}

bool RobotModel::IntakeSensorActivated(){
	bool noObstruction = false;
	if (intakeSensor->Get() == 0) {
		return noObstruction;
	}
	else return !noObstruction;
}

/**
void RobotModel::PrintToLCD(int line, char message) {
	switch(line) {
	Line lineNum;
	case 1:
		lineNum = DriverStationLCD::kUser_Line1;
		break;
	case 2:
		lineNum = DriverStationLCD::kUser_Line2;
		break;
	case 3:
		lineNum = DriverStationLCD::kUser_Line3;
		break;
	case 4:
		lineNum = DriverStationLCD::kUser_Line4;
		break;
	case 5:
		lineNum = DriverStationLCD::kUser_Line5;
		break;
	case 6:
		lineNum = DriverStationLCD::kUser_Line6;
		break;
	default:
		lineNum = DriverStationLCD::kUser_Line1;
		break;
	}
		dsLCD->Printf(lineNum, 1, message);
		dsLCD->UpdateLCD();
}
**/

void RobotModel::ResetGyro()  {
	gyro->Reset();
}

float RobotModel::GetGyroAngle()  {
	//return gyro->GetAngle();
	return 0.0;
}

bool RobotModel::IsLowGear()  {
	return isLowGear;
}

void RobotModel::ShiftToLowGear() {
	shifterSolenoid->Set(true);
	isLowGear = true;
	// printf("Shifting to low gear: Solenoid %d is now TRUE\n", GEAR_SHIFT_SOLENOID_CHAN);
}

void RobotModel::ShiftToHighGear() {
	shifterSolenoid->Set(false);
	isLowGear = false;
	// printf("Shifting to high gear: Solenoid %d is now FALSE\n", GEAR_SHIFT_SOLENOID_CHAN);
}

void RobotModel::ShiftGear()  {
	if (isLowGear) {
		ShiftToHighGear();
	} else {
		ShiftToLowGear();
	}
}
//ramp up is modeled after gear shift.
bool RobotModel::RampArmIsUp()  {
	return rampArmIsUp;
}

bool RobotModel::RampLegsAreUp() {
	return rampLegsAreUp;
}

void RobotModel::ManipRamp()  {
	// not used anymore
	
	/*if (rampArmIsUp) {
		LowerRampManipulator();
	} else {
		RaiseRampManipulator();
	}*/
}

void RobotModel::RaiseRampLegs() {
	rampLegsSolenoid->Set(false); // May be true!! I have no idea!! Must be empirically tested.
	rampLegsAreUp = true;
	printf("raising ramp legs\n");
}

void RobotModel::LowerRampLegs() {
	rampLegsSolenoid->Set(true); // May be false!! I have no idea!! Must be empirically tested.	
	rampLegsAreUp = false;
	printf("lowering ramp legs\n");
}

void RobotModel::RaiseIntake() {
	rampArmUpSolenoid->Set(true);
	rampArmDownSolenoid->Set(false);
	//RaiseRampLegs();			  // Raise the legs, too, just in case. 
	rampArmIsUp = true;
	printf("raising intake\n");
}

void RobotModel::LowerIntake() {
	rampArmUpSolenoid->Set(false);
	rampArmDownSolenoid->Set(true);
	rampArmIsUp = false;
	printf("lowering intake\n");
}

void RobotModel::ToggleIntake() {
	if (rampArmIsUp) {
		LowerIntake();
	} else {
		RaiseIntake();
	}
}

bool RobotModel::IsHoodLowered()  {
	return hoodLowered;
}

void RobotModel::ToggleHood(){
	if (hoodLowered) {
		RaiseHood();
		printf("Lower hood: Solenoid %d is now FALSE\n", HOOD_ADJUST_SOLENOID_CHAN);
	} else {
		LowerHood();
		printf("Raise hood: Solenoid %d is now TRUE\n", HOOD_ADJUST_SOLENOID_CHAN);
	}
}

void RobotModel::LowerHood(){
	hoodLowered = true;
	hoodSolenoid->Set(hoodLowered);
}

void RobotModel::RaiseHood(){
	hoodLowered = false;
	hoodSolenoid->Set(hoodLowered);
}

void RobotModel::TogglePopUp(){
	if (popUpEngaged) {
		DisengagePopUp();
		// printf("Disengaging pop-up: Solenoid %d is now FALSE\n", POP_UP_SOLENOID_CHAN);
	} else {
		EngagePopUp();
		// printf("Engaging pop-up: Solenoid %d is now TRUE\n", POP_UP_SOLENOID_CHAN);
	}
}

void RobotModel::EngagePopUp(){
	popUpEngaged = true;
	popUpSolenoid->Set(popUpEngaged);
}

void RobotModel::DisengagePopUp(){
	popUpEngaged = false;
	popUpSolenoid->Set(popUpEngaged);
}

double RobotModel::GetWheelEncoderDistance(Wheels w) {
	switch (w) {
		case kLeftWheel:
			return ( leftWheelEncoder->Get() / 275.0 ) ;
		case kRightWheel:
			return ( -(rightWheelEncoder->Get()) / 275.0 );  //was 142, need 275 for 256 count encoders
		default: return 0.0;
	}
}


double RobotModel::GetWheelEncoderValue(Wheels w) {
	switch (w) {
	case kLeftWheel:
		return leftWheelEncoder->Get();
	case kRightWheel:
		return -(rightWheelEncoder->Get());
	default: return 0.0;
	}
}

double RobotModel::GetRightWheelEncoderValue()  {
	return(-rightWheelEncoder->Get());
}

double RobotModel::GetLeftWheelEncoderValue()  {
	return(leftWheelEncoder->Get());
}

//shooter
double RobotModel::GetShooterEncoderValue()  {
	return(shooterEncoder->Get());
}

double RobotModel::GetConveyorEncoderValue()  {
	return(conveyorEncoder->Get());
}

void RobotModel::ResetWheelEncoder(Wheels w/* = kBothWheels*/) {
	switch (w) {
	case kLeftWheel:
		leftWheelEncoder->Reset();
		break;
	case kRightWheel:
		rightWheelEncoder->Reset();
		break;
	case kBothWheels:
		leftWheelEncoder->Reset();
		rightWheelEncoder->Reset();
		break;
	default: break;
	}
}

void RobotModel::ResetShooterEncoder(){
	shooterEncoder->Reset();
}

void RobotModel::ResetConveyorEncoder(){
	conveyorEncoder->Reset();
}

// Note: Pass this function positive values to move the robot forward,
// negative values to move the robot backwards. The function handles
// reversing the right side.

void RobotModel::SetWheelSpeed(double speed, Wheels w) {
	if (!brakeIsOn) {
		switch (w) {
		case kLeftWheel:
			driveVictorLeftA->Set(-speed);
			driveVictorLeftB->Set(-speed);
			driveVictorLeftC->Set(-speed);
			break;
		case kRightWheel:
			driveVictorRightA->Set(speed);
			driveVictorRightB->Set(speed);
			driveVictorRightC->Set(speed);

 			break;
		case kBothWheels:
			driveVictorLeftA-> Set(-speed);
			driveVictorLeftB-> Set(-speed);
			driveVictorLeftC-> Set(-speed);
			driveVictorRightA->Set(speed);
			driveVictorRightB->Set(speed);
			driveVictorRightC->Set(speed);
			break;
		default: break;
		}
	} else { // Brake is engaged, so we don't want to power the motors.
		DO_PERIODIC(5000, printf("Brake engaged; motors disabled."));
		driveVictorLeftA-> Set(0);
		driveVictorLeftB-> Set(0);
		driveVictorLeftC-> Set(0);
		driveVictorRightA->Set(0);
		driveVictorRightB->Set(0);
		driveVictorRightC->Set(0);
	}
}

void RobotModel::SetFlywheelSpeed(double speed) {
	flywheelVictorA->Set(speed); // if we're on the practice bot, this needs to be negative
	flywheelVictorB->Set(speed);
	if (fabs(speed) > 0) isFlywheelOn = true;
	else isFlywheelOn = false;
}

// FISHER PRICE needs negative speed
// BaneBots needs +ve
void RobotModel::SetRollerSpeed(double speed) {
	rollerVictor->Set(speed);
}

void RobotModel::SetConveyorSpeed(double speed) {
	conveyorVictor->Set(speed);
}

void RobotModel::SetBrakeOn() {
	brakeOnSolenoid->Set(true);
	brakeOffSolenoid->Set(false);
	brakeIsOn = true;
	printf("Setting brake on:\n  Solenoid %d is now TRUE\n  Solenoid %d is now FALSE\n", BRAKE_ON_SOLENOID_CHAN, BRAKE_OFF_SOLENOID_CHAN);
}

void RobotModel::SetBrakeOff() {
	brakeOnSolenoid->Set(false);
	brakeOffSolenoid->Set(true);
	brakeIsOn = false;
	printf("Setting brake off:\n  Solenoid %d is now FALSE\n  Solenoid %d is now TRUE\n", BRAKE_ON_SOLENOID_CHAN, BRAKE_OFF_SOLENOID_CHAN);
}

void RobotModel::ToggleBrake() {
	if (brakeIsOn) {
		SetBrakeOff();
	} else {
		SetBrakeOn();
	}
}

bool RobotModel::BrakeIsOn() {
	return brakeIsOn;
}

#ifdef USE_CAMERA
HSLImage* RobotModel::GetCameraImage() {
	if (camera == NULL) return NULL;
	return camera->GetImage();
}
#endif

double RobotModel::GetCurrentTimeInSeconds() {
	return timer->Get();
}

RobotModel::~RobotModel()
{
}
