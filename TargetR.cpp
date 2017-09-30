#include "TargetR.h"
#include "WPILib.h"

static RectangleDescriptor rectangleDescriptor = {
										3, 		// minWidth
										200, 	// maxWidth
										3, 		// minHeight
										100		// maxHeight
										};

static CurveOptions curveOptions = {	IMAQ_NORMAL_IMAGE,	// extractionMode
										40, 				// threshold
										IMAQ_NORMAL, 		// filterSize
										25, 				// minLength
										15, 				// rowStepSize 
										15, 				// columnStepSize
										10, 				// maxEndPointGap
										1,					// onlyClosed
										0					// subpixelAccuracy
										};

static ShapeDetectionOptions shapeOptions = {
										IMAQ_GEOMETRIC_MATCH_SHIFT_INVARIANT,	// mode
										NULL,			// angle ranges
										0,				// num angle ranges
										{75, 125},		// scale range
										500				// minMatchScore
};



TargetR::TargetR(){
}

vector<RectangleMatch> TargetR::FindTargets(HSLImage *image) {
	//vector<Target> targets;
	
	MonoImage  *luminancePlane = image->GetLuminancePlane();
	
	Image* imaqImage = luminancePlane->GetImaqImage();
	int numMatches;
	
	RectangleMatch* rectangles = imaqDetectRectangles(imaqImage, &rectangleDescriptor, &curveOptions, &shapeOptions, NULL, &numMatches);
	
	vector<RectangleMatch> *results;
	if (rectangles == NULL) {
		return *results;
	}
	else {
		for (int i=0; i<numMatches; i++) {
			results->push_back(rectangles[i]);
		}
	}
	imaqDispose(rectangles);
	
	return *results;
}

TargetR::~TargetR(){
}
