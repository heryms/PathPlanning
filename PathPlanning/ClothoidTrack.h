#pragma once
#ifndef _PATH_PLANNING_CLOTHOID_TRACK_H
#define _PATH_PLANNING_CLOTHOID_TRACK_H
#include "BaseTrack.h"
class ClothoidTrack :public BaseTrack
{
public:
	ClothoidTrack();
	~ClothoidTrack();
	void Track();
};

#endif // !_PATH_PLANNING_CLOTHOID_TRACK_H