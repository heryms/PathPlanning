#ifndef _PATH_GENERATE_TOOL_H
#define _PATH_GENERATE_TOOL_H
#include "BaseType.h"
#include "Topology.h"
#include <Eigen/Dense>
class SXYSpline {
public:
	Eigen::MatrixX4d skX;
	Eigen::MatrixX4d skY;
	std::vector<double> S;//累积弧长
	std::vector<double> X;//点X
	std::vector<double> Y;//点Y
	int splineNum;
	void init(std::vector<RoadPoint> baseFrame);
	double getKappa(double Sf);
	RadAngle getTangent(double Sf);
	void getDeriveXY(double Sf, double & dx, double & dy);
	void getXY(double Sf, double & X, double & Y);
	double getS(int index, double X, double Y);
};
#endif