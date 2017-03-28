#pragma once
#ifndef _PATH_PLANNING_BASE_TRACK_H
#define _PATH_PLANNING_BASE_TRACK_H

#include <vector>
#include "BaseType.h"
class BaseTrack
{
protected:
	std::vector<RoadPoint> path;
	double refSpeedStraight = 20;
	double refSpeedCurve = 10;
	bool inCurve = false;
public:
	BaseTrack();
	~BaseTrack();
	virtual void SetPath(std::vector<RoadPoint>& path);
	/**
	* @param straight 直道参考车速km/h
	* @param curve 弯道参考车速km/h
	**/
	virtual void SetRefSpeed(double straight, double curve);
	virtual void Track()=0;
};

#endif // !_PATH_PLANNING_BASE_TRACK_H
