#pragma once
#ifndef _PATH_PLANNING_BASE_TRACK_H
#define _PATH_PLANNING_BASE_TRACK_H

#include <vector>
#include "BaseType.h"
class BaseTrack
{
protected:
	std::vector<RoadPoint> path;
	double refSpeedStraight = 30;
	double refSpeedCurve = 5;
	bool inCurve = false;
	double recommendSpeed;
public:
	bool b_stop;
	BaseTrack();
	~BaseTrack();
	virtual void SetLocalPath(std::vector<RoadPoint>& path);
	/**
	* @param straight 直道参考车速km/h
	* @param curve 弯道参考车速km/h
	**/
	virtual void SetRefSpeed(double straight, double curve);
	virtual void Track()=0;
	virtual void SetRecommendSpeed(double speed){
		recommendSpeed = speed;
	}
};

#endif // !_PATH_PLANNING_BASE_TRACK_H
