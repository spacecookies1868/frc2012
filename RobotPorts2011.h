#ifndef ROBOT_PORTS_2011_H
#define ROBOT_PORTS_2011_H

static const int LEFT_MOTOR_A_PWM_PORT = 3;
static const int LEFT_MOTOR_B_PWM_PORT = 4;

static const int RIGHT_MOTOR_A_PWM_PORT = 6;
static const int RIGHT_MOTOR_B_PWM_PORT = 5;

//static const int ARM_MOTOR_A_PWM_PORT = 1;
//static const int ARM_MOTOR_B_PWM_PORT = 2;

//shooter
static const int SHOOTER_MOTOR_A_PWM_PORT = 1;
static const int SHOOTER_MOTOR_B_PWM_PORT = 2;

static const int LEFT_WHEEL_ENCODER_A_PWM_PORT = 3; //encoders on digital ports
static const int LEFT_WHEEL_ENCODER_B_PWM_PORT = 4;

static const int RIGHT_WHEEL_ENCODER_A_PWM_PORT = 5;
static const int RIGHT_WHEEL_ENCODER_B_PWM_PORT = 6;

//static const int ARM_ENCODER_A_PWM_PORT = 13;
//static const int ARM_ENCODER_B_PWM_PORT = 14;

//shooter
static const int SHOOTER_ENCODER_A_PWM_PORT = 13;
static const int SHOOTER_ENCODER_B_PWM_PORT = 14;

static const int GYRO_PORT = 1;	//analog port

static const int GEAR_SHIFT_SOLENOID_CHAN=3;
static const int CLAW_OPEN_SOLENOID_CHAN = 1;
static const int CLAW_CLOSED_SOLENOID_CHAN = 6;
static const int ARM_RETRACTED_SOLENOID_CHAN = 4;
static const int ARM_EXTENDED_SOLENOID_CHAN = 5;
static const int PROP_UP_SOLENOID_CHAN = 7;
//static const int MINIBOT_DEPLOY_SOLENOID_CHAN = 2;
//static const int MINIBOT_LOWER_SOLENOID_CHAN = 7;
//static const int MINIBOT_RAISE_SOLENOID_CHAN = 8;

static const int COMPRESSOR_PRESSURE_SWITCH_CHAN=1;
static const int COMPRESSOR_RELAY_CHAN=1;

static const int LEFT_JOY_USB_PORT = 1;
static const int RIGHT_JOY_USB_PORT = 2;
//shooter
static const int SHOOTER_JOY_USB_PORT = 3;

#endif
