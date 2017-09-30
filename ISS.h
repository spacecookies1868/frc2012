#ifndef ISS_H_
#define ISS_H_

#include "RobotModel.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "ShooterController.h"

class ISS{
public: 
	ISS(RobotModel*, RemoteController*, DriveController*, ShooterController*);
	virtual ~ISS();
	void Update(double currTimeSec, double deltaTimeSec);
	
	enum ISSstate{
		kIdle, kDriveForward, kDriveForwardDone, kAim, kAimDone, kShoot, kReset
	};
private:
	RobotModel *robot;
	RemoteController *humanControl;
	DriveController *driveControl;
	ShooterController *shooterControl;
	
	uint32_t m_stateVal;
	uint32_t nextState;
};
#endif /*ISS_H_*/
