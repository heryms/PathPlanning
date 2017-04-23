#pragma once
#ifndef _PATH_PLANNING_PURE_TRACK_H
#define _PATH_PLANNING_PURE_TRACK_H
#include <vector>
#include "BaseType.h"
#include "BaseTrack.h"

class PureTrack : public BaseTrack
{
public:
	PureTrack();
	~PureTrack();
	void Track();
};


#endif // !_PATH_PLANNING_PURE_TRACK_H