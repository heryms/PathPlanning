#pragma once
#ifndef _PATH_PLANNING_TRACK_HELPER_H
#define _PATH_PLANNING_TRACK_HELPER_H
#include "BaseType.h"
#include <vector>
#include "PureTrack.h"
class TrackThread;
class BaseTrack;
class TrackHelper
{
private:
	friend class TrackThread;
	//std::vector<RoadPoint> path;
	//std::mutex m_lock;
	PureTrack* track;
	//BaseTrack* track;
	TrackThread* thread;
	double m_recommendSpeed;
public:
	TrackHelper();
	virtual ~TrackHelper();
	void SetLocalPath(std::vector<RoadPoint>& path);
	void SetPath(std::vector<RoadPoint>& path);
	void SetRecommendSpeed(double speed);
	void Track();
	void Start();
	void Suspend();
	void Resume();
	void End();
	void Stop()
	{
		track->b_stop = true;
	}
	void ReStart()
	{
		track->b_stop = false;
	}
};

#endif // !_PATH_PLANNING_BASE_TRACK_H