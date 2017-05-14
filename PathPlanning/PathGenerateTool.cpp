#include "PathGenerateTool.h"

void SXYSpline::init(std::vector<RoadPoint> baseFrame)
{
	X.clear();
	Y.clear();
	double s = 0;
	X.push_back(baseFrame.begin()->x);
	Y.push_back(baseFrame.begin()->y);
	S.push_back(s);
	for (auto pt = baseFrame.begin() + 1; pt != baseFrame.end(); pt++) {
		X.push_back(pt->x);
		Y.push_back(pt->y);
		double dy = pt->y - (pt - 1)->y;
		double dx = pt->x - (pt - 1)->x;
		s += sqrt(dx*dx + dy*dy);
		S.push_back(s);
	}
	double dx0, dxn = 0;
	double dy0, dyn = 0;
	if (X.size() >= 2) {
		dx0 = (X[1] - X[0]) / (S[1] - S[0]);
		dxn = (*X.rbegin() - *(X.rbegin() + 1)) / (*S.rbegin() - *(S.rbegin() + 1));
	}
	if (Y.size() >= 2) {
		dy0 = (Y[1] - Y[0]) / (S[1] - S[0]);
		dyn = (*Y.rbegin() - *(Y.rbegin() + 1)) / (*S.rbegin() - *(S.rbegin() + 1));
	}
	skX = Topology::CubicSpline(S, X, dx0, dxn);
	skY = Topology::CubicSpline(S, Y, dy0, dyn);
	splineNum = skX.rows();
}



double SXYSpline::getKappa(double Sf)
{
	int index = std::find_if(S.begin(), S.end(), 
		[Sf](double d)->bool {
			return d > Sf;
	}) - S.begin() - 1;
	Eigen::Vector4d kx = skX.row(index);
	Eigen::Vector4d ky = skY.row(index);
	double ds = Sf - S[index];
	double dx = 3 * kx(3)*ds*ds + 2 * kx(2)*ds + kx(1);
	double ddx = 6 * kx(3)*ds + 2 * kx(2);
	double dy = 3 * ky(3)*ds*ds + 2 * ky(2)*ds + ky(1);
	double ddy = 6 * ky(3)*ds + 2 * ky(2);
	return abs(dx*ddy-ddx*dy)/pow(dx*dx+dy*dy,3/2);
}

RadAngle SXYSpline::getTangent(double Sf)
{
	int index = std::find_if(S.begin(), S.end(), [Sf](double d)->bool {
		return d > Sf;
	}) - S.begin() - 1;
	Eigen::Vector4d kx = skX.row(index);
	Eigen::Vector4d ky = skY.row(index);
	double ds = Sf - S[index];
	double dx = 3 * kx(3)*ds*ds + 2 * kx(2)*ds + kx(1);
	double dy = 3 * ky(3)*ds*ds + 2 * ky(2)*ds + ky(1);
	return atan(dy / dx);
}

void SXYSpline::getDeriveXY(double Sf, double & dx, double & dy)
{
	int index = std::find_if(S.begin(), S.end(), [Sf](double d)->bool {
		return d > Sf;
	}) - S.begin() - 1;
	Eigen::Vector4d kx = skX.row(index);
	Eigen::Vector4d ky = skY.row(index);
	double ds = Sf - S[index];
	dx = 3 * kx(3)*ds*ds + 2 * kx(2)*ds + kx(1);
	dy = 3 * ky(3)*ds*ds + 2 * ky(2)*ds + ky(1);
}

void SXYSpline::getXY(double Sf, double & X, double & Y)
{
	int index = std::find_if(S.begin(), S.end(), [Sf](double d)->bool {
		return d > Sf;
	}) - S.begin() - 1;
	Eigen::Vector4d kx = skX.row(index);
	Eigen::Vector4d ky = skY.row(index);
	double ds = Sf - S[index];
	X = kx(3)*pow(ds, 3) + kx(2)*ds*ds + kx(1)*ds + kx(0);
	Y = ky(3)*pow(ds, 3) + ky(2)*ds*ds + ky(1)*ds + ky(0);
}

double SXYSpline::getS(int index, double X, double Y)
{
	Eigen::Vector4d kx = skX.row(index);
	Eigen::Vector4d ky = skY.row(index);
	double a = (kx(2)*ky(3) - kx(3)*ky(2));
	double b = (kx(1)*ky(3) - kx(3)*ky(1));
	double c = (kx(0)*ky(3) - kx(3)*ky(0)) - (ky(3)*X - kx(3)*Y);
	double ret0 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
	double ret1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
	if (ret0 >= S[index] && (index == splineNum - 1 || ret0 <= S[index + 1])) {
		return ret0;
	}
	if (ret1 >= S[index] && (index == splineNum - 1 || ret1 <= S[index + 1])) {
		return ret1;
	}
	return -1;
}
