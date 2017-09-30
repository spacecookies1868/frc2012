#ifndef ROBOTPORTS2012_H_
#define ROBOTPORTS2012_H_

//********PWM PORTS (CORRECT FOR 2012 ROBOT)**********
static const int CONVEYOR_MOTOR_PWM_PORT		= 1;
static const int ROLLER_MOTOR_PWM_PORT 			= 2;

static const int RIGHT_MOTOR_A_PWM_PORT 		= 3; 
static const int RIGHT_MOTOR_B_PWM_PORT 		= 4;
static const int RIGHT_MOTOR_C_PWM_PORT 		= 5;

static const int LEFT_MOTOR_A_PWM_PORT 			= 6;
static const int LEFT_MOTOR_B_PWM_PORT 			= 7;
static const int LEFT_MOTOR_C_PWM_PORT 			= 8;

static const int FLYWHEEL_MOTOR_A_PWM_PORT 		= 9;
static const int FLYWHEEL_MOTOR_B_PWM_PORT 		= 10; 

//***********DIGITAL I/O PORTS************************
static const int GYRO_PORT 						= 1; 

static const int LEFT_WHEEL_ENCODER_A_PWM_PORT 	= 2;
static const int LEFT_WHEEL_ENCODER_B_PWM_PORT 	= 3;

static const int RIGHT_WHEEL_ENCODER_A_PWM_PORT = 4;
static const int RIGHT_WHEEL_ENCODER_B_PWM_PORT = 5;

static const int CONVEYOR_ENCODER_A_PWM_PORT 	= 6;
static const int CONVEYOR_ENCODER_B_PWM_PORT 	= 7;

static const int FLYWHEEL_ENCODER_A_PWM_PORT 	= 8;
static const int FLYWHEEL_ENCODER_B_PWM_PORT 	= 9;

static const int BEAM_SENSOR_PORT 				= 10;
static const int INTAKE_SENSOR_PORT				= 11;

//*********SOLENOID PORTS****************************
static const int RAMP_ARM_DOWN_SOLENOID_CHAN	= 1;
static const int RAMP_ARM_UP_SOLENOID_CHAN		= 4;

static const int BRAKE_ON_SOLENOID_CHAN 		= 2;
static const int BRAKE_OFF_SOLENOID_CHAN		= 3;

static const int POP_UP_SOLENOID_CHAN			= 5;

static const int GEAR_SHIFT_SOLENOID_CHAN		= 6;

static const int HOOD_ADJUST_SOLENOID_CHAN		= 7;

static const int RAMP_LEGS_SOLENOID_CHAN		= 8; // not sure about this number

//**************MISC**********************************
static const int COMPRESSOR_PRESSURE_SWITCH_CHAN = 1;
static const int COMPRESSOR_RELAY_CHAN 			= 1;

//**********JOYSTICK USB PORTS************************
static const int LEFT_JOY_USB_PORT 				= 1;
static const int RIGHT_JOY_USB_PORT 			= 2;
static const int OPERATOR_JOY_USB_PORT 			= 3;
static const int ISS_JOY_USB_PORT				= 4;

#endif /*ROBOTPORTS2012_H_*/
