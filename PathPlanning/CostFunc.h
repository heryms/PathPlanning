#ifndef _PATH_GENERATE_COST_FUNCTION_H
#define _PATH_GENERATE_COST_FUNCTION_H

#include "BaseType.h"
#include "Clothoid.h"
#include "CoordTransform.h"
double cost(std::vector<RoadPoint> now_cur, std::vector<RoadPoint> reference_map, std::vector<double> guassian_prob);
double similarity(std::vector<RoadPoint> pre_cur, std::vector<RoadPoint> now_cur);
double similarity(std::vector<RoadPoint> pre_cur, std::vector<RoadPoint> now_cur, double sf);
#endif