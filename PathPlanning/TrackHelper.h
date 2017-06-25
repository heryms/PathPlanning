#pragma once
#ifndef _PATH_PLANNING_TRACK_HELPER_H
#define _PATH_PLANNING_TRACK_HELPER_H
#include "BaseType.h"
#include <vector>
class TrackThread;
class BaseTrack;
class TrackHelper
{
private:
	friend class TrackThread;
	//std::vector<RoadPoint> path;
	//std::mutex m_lock;
	//PureTrack* track;
	BaseTrack* track;
	TrackThread* thread;
public:
	TrackHelper();
	virtual ~TrackHelper();
	void SetLocalPath(std::vector<RoadPoint>& path);
	void SetPath(std::vector<RoadPoint>& path);
	void Track();
	void Start();
	void Suspend();
	void Resume();
	void End();
};

#endif // !_PATH_PLANNING_BASE_TRACK_H