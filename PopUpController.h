#ifndef POPUPCONTROLLER_H_
#define POPUPCONTROLLER_H_

#include "RobotModel.h"
#include "RemoteControl.h"

class PopUpController
{
public:
	PopUpController(RobotModel*, RemoteController*);
	virtual ~PopUpController();
	void Update(double currTimeSec, double deltaTimeSec);
	
	enum PopUpState {
		kInitialize, kEngaged, kDisengaged
	};

private:
	RobotModel *robot;
	RemoteController *humanControl;

	uint32_t m_stateVal;
	uint32_t nextState;
};
#endif /*POPUPCONTROLLER_H_*/
