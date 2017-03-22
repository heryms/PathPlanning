#ifndef _PATH_GENERATE_H
#define _PATH_GENERATE_H
#include "BaseType.h"
#include "Clothoid.h"
#include "CoordTransform.h"
#include "DataCenter.h"
#include <vector>
#include "PathDraw.h"
class PathGenerate
{
private:
	PathDraw m_sendPath;
public:
	PathGenerate()
	{

	}

	~PathGenerate()
	{

	}
	bool path_generate_grid(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids,std::vector<RoadPoint>& genPoints);
	bool path_generate_local(PosPoint startPt, PosPoint endPt);
	void path_generate();
	int getRightestPoints(RoadPoint *rdPt, int num_Pt);
	double getTargetDirection();
	

private:
	double target_X;
	double target_Y;
};


#endif