#include "PopUpController.h"
#include "RobotModel.h"

PopUpController::PopUpController(RobotModel *myRobot, RemoteController *myHumanControl)
{
	robot = myRobot;
	humanControl = myHumanControl;
	m_stateVal = kInitialize;
	//pop up automatically goes to disengaged state (or should it be reset state?)
}

void PopUpController::Update(double currTimeSec, double deltaTimeSec){
	
	switch(m_stateVal){
	
	case (kInitialize):
	      robot->DisengagePopUp();
	      nextState = kDisengaged;
	      break;
	  
	  case(kEngaged):
	      robot->EngagePopUp();
	      if(robot->BeamSensorActivated()){
	        nextState = kDisengaged;
	        printf("Beam sensor activated, so disengaging pop-up.\n");
	        //robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Popup OFF" );
	    	//robot->dsLCD->UpdateLCD();
	      } else if (humanControl->PopUpDesired()) {
	    	 nextState = kDisengaged;
	    	 printf("Human desires pop-up disengaged, so disengaging popup.\n");
		    // robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Popup OFF" );
		   //  robot->dsLCD->UpdateLCD();
	      }
	      break;
	  
	  case(kDisengaged):
	      robot->DisengagePopUp();
	  	  if (humanControl->PopUpDesired()){
	  	        nextState = kEngaged;
	  	        printf("Human desires pop-up, so engaging pop-up.\n");
	  	        //robot->dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Popup ON" );
	  	      	//robot->dsLCD->UpdateLCD();
	  	  }
	      break;
	
	}
	
	m_stateVal = nextState;

}

PopUpController::~PopUpController()
{
}
