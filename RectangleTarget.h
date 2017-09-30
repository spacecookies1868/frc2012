#ifndef RECTANGLETARGET_H_
#define RECTANGLETARGET_H_

#import "WPILib.h"

class RectangleTarget
{
public:
	double width;
	double height;
	double xPos;
	double yPos;
	double xCenter;
	double yCenter;
	double score;
	
	RectangleTarget();
	
	static vector<RectangleMatch> FindTargets(HSLImage *image);
	
	virtual ~RectangleTarget();
};

#endif /*RECTANGLETARGET_H_*/
