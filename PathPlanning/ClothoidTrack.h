#pragma once
#ifndef _PATH_PLANNING_CLOTHOID_TRACK_H
#define _PATH_PLANNING_CLOTHOID_TRACK_H
#include "BaseType.h"
#include <vector>
#include <mutex>
class TrackThread;
class ClothoidTrack
{
private:
	friend class TrackThread;
	std::vector<RoadPoint> path;
	std::mutex m_lock;
	TrackThread* thread;
public:
	ClothoidTrack();
	virtual ~ClothoidTrack();
	void SetPath(std::vector<RoadPoint> path);
	void Track(double k);
	void Start();
	void Suspend();
	void Resume();
	void End();
};

#endif // !_PATH_PLANNING_BASE_TRACK_H