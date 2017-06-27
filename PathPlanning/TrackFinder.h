#pragma once
#ifndef _PATH_PLANNING_TRACK_FINDER_H
#define _PATH_PLANNING_TRACK_FINDER_H
#include "BaseType.h"
#include <vector>
class TrackFinder
{
public:
	TrackFinder();
	~TrackFinder();
	static int FindPointIndex(std::vector<RoadPoint>& path, RoadPoint curX);
	static int AnchorPoint(std::vector<RoadPoint>& path, RoadPoint curX, int cuIndex = -1, double distance = 5);
	static bool InCurve(bool isInCurve, std::vector<RoadPoint>& path, RoadPoint curX, int curIndex);
	static double calulate_radios_err(double x, double y, double radios);
static	RoadPoint Newton_divide(RoadPoint p0, RoadPoint p1, float radious);
//static bool FindPursuitPoint(vector<NodePoint> linebuffer, RoadPoint curPoint, RoadPoint &TargetPoint, float Speed);
static bool FindPursuitPoint(std::vector<RoadPoint>& refPath, RoadPoint curX, RoadPoint &refX, float Speed);
static short MiddlePiont_Shan(PosPoint org, double firMidPoint_x, double firMidPoint_y);
static
		int AnchorPointStraight(std::vector<RoadPoint>& path, RoadPoint curX, int curIndex, double distance);
};


#endif // !_PATH_PLANNING_TRACK_FINDER_H

