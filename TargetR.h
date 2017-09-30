#ifndef TARGETR_H_
#define TARGETR_H_

#import "WPILib.h"

class TargetR
{
public:
	TargetR();
	
	static vector<RectangleMatch> FindTargets(HSLImage *image);
	
	virtual ~TargetR();
};

#endif /*TARGETR_H_*/
