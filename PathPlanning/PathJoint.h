#ifndef _PATH_GENERATE_PATH_JOINT_H_
#define _PATH_GENERATE_PATH_JOINT_H_
#include <BaseType.h>
#include <vector>
#include "Topology.h"

class PathJoint{
public:
	static std::vector<RoadPoint> Decare_To_Gaussian(std::vector<RoadPoint> root, double theta, double x_car, double y_car);
	static std::vector<RoadPoint> Gaussian_To_Decare(std::vector<RoadPoint> root, double theta, double x_car, double y_car);
};
#endif


