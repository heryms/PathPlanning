#ifndef _PATH_GENERATE_H
#define _PATH_GENERATE_H
#include "BaseType.h"
#include "Clothoid.h"
#include "CoordTransform.h"
#include "DataCenter.h"
#include "Topology.h"
#include "CostFunc.h"
#include <vector>
#include <queue>
#include "PathDraw.h"
#include "CarControl.h"
#include "TrackHelper.h"
class PathGenerate
{
private:
	TrackHelper track;
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
	bool path_generate_grid_obstacle(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids, std::vector<RoadPoint>& genPoints, int *y, int *x);
	void path_generate();
	void path_generate_using_bug();
	std::vector<RoadPoint> getRdPtFromTable(int grid, int angle);
	int getRightestPoints(RoadPoint *rdPt, int num_Pt);
	double getTargetDirection();
	bool path_generate_turning(VeloGrid_t& veloGrids);
	bool path_generate_for_fun();
	bool path_generate_recursive(PosPoint startPt, PosPoint endPt, VeloGrid_t veloGrids, 
		std::vector<PosPoint> &root, int count);

private:
	std::vector<RoadPoint> pre_Root;
	std::vector<std::vector<PosPoint>> posPtOnRoot;
	std::vector<std::vector<RoadPoint>> clothoidMap;
	double PreDirection;
	double target_X;
	double target_Y;
	std::queue<double> recAngle;

};


#endif