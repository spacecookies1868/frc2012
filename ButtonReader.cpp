#include "ButtonReader.h"

ButtonReader::ButtonReader(Joystick* myJoystick, int myButtonNum) {
	joystick = myJoystick;
	buttonNum = myButtonNum;
	currState = joystick->GetRawButton(buttonNum);
	lastState = currState;
}

bool ButtonReader::WasJustPressed() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
	return (lastState == false && currState == true);
}

bool ButtonReader::WasJustReleased() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
	return (lastState == true && currState == false);
}

bool ButtonReader::StateJustChanged() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
	return (lastState != currState);
}

bool ButtonReader::IsDown() {
	return (joystick->GetRawButton(buttonNum));
}

ButtonReader::~ButtonReader()
{
}
