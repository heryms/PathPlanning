#include "PathJoint.h"

std::vector<RoadPoint> PathJoint::Decare_To_Gaussian(std::vector<RoadPoint> root, double theta, double x_car, double y_car){
	std::vector<RoadPoint> pts_Gaussian;
	for (auto pt:root)
	{
		double x, y;
		Topology::Rotate_To_Guassian(theta, pt.x, pt.y, x, y);
		RoadPoint pt_g;
		pt_g.x = x + x_car;
		pt_g.y = y + y_car;
		pts_Gaussian.push_back(pt_g);
	}
	return pts_Gaussian;
}

std::vector<RoadPoint> PathJoint::Gaussian_To_Decare(std::vector<RoadPoint> root, double theta, double x_car, double y_car){
	std::vector<RoadPoint> pts_Decare;
	for (auto pt : root)
	{
		double x, y;
		
		Topology::Rotate_To_Guassian(theta, pt.x-x_car, pt.y-y_car, x, y);
		RoadPoint pt_d;
		pt_d.x = x;
		pt_d.y = y;
		pts_Decare.push_back(pt_d);

	}
	return pts_Decare;
}