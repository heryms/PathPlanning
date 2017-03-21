#ifndef _PATH_GENERATE_H
#define _PATH_GENERATE_H
#include "BaseType.h"
#include "Clothoid.h"
#include "CoordTransform.h"
class PathGenerate
{
public:
	PathGenerate()
	{
	}

	~PathGenerate()
	{
	}
	bool path_generate_grid(PosPoint startPt, PosPoint endPt);
	bool path_generate_local(PosPoint startPt, PosPoint endPt);
	void path_generate();
	int getRightestPoints(RoadPoint *rdPt, int num_Pt);
	double getTargetDirection();
	

private:
	double target_X;
	double target_Y;
	double 
};


#endif