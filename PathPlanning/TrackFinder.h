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
};


#endif // !_PATH_PLANNING_TRACK_FINDER_H

