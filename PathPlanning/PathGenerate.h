#ifndef _PATH_GENERATE_H
#define _PATH_GENERATE_H
#include "BaseType.h"
#include "Clothoid.h"
#include "CoordTransform.h"
#include "DataCenter.h"
#include <vector>
#include <queue>
#include "PathDraw.h"
#include "CarControl.h"
#include "ClothoidTrack.h"
class PathGenerate
{
private:
	ClothoidTrack track;
	PathDraw m_sendPath;
public:
	PathGenerate()
	{
		createClothoidTable();
		track.Start();
	}

	~PathGenerate()
	{
		track.End();
	}
	void createClothoidTable();
	void generateClothoidPoints(PosPoint startPt, PosPoint endPt, std::vector<RoadPoint>& genPoints);
	bool path_generate_grid(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids,std::vector<RoadPoint>& genPoints);
	bool path_generate_local(PosPoint startPt, PosPoint endPt);
	bool path_generate_grid_obstacle(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids, std::vector<RoadPoint>& genPoints);
	void path_generate();
	std::vector<RoadPoint> getRdPtFromTable(int grid, int angle);
	int getRightestPoints(RoadPoint *rdPt, int num_Pt);
	double getTargetDirection();
	

private:
	std::vector<std::vector<RoadPoint>> clothoidMap;
	double target_X;
	double target_Y;
	std::queue<double> recAngle;
};


#endif