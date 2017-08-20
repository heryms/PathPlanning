#pragma once

#include "lcmtype\VeloGrid_t.hpp"
#include "BaseType.h"
using namespace ckLcmType;

class CollisionDetection
{
public:
	//collision check for a single path
	static int collisionCheck(VeloGrid_t& grids, std::vector<RoadPoint>& localPath, bool hasAngle = false);
	//detect if a single point collides with the velogrid data
	static bool detectObstacle(RoadPoint cur, VeloGrid_t & grids, RoadPoint &collisionpoint);
};

