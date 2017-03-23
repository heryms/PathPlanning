/*
	algorithms for clothoid curve
	author: heryms
	created: 2017-3-20

*/
#ifndef _CLOTHOID_PATH_GENERATE
#define _CLOTHOID_PATH_GENERATE
#include <cmath>
#include "Variables.h"
#include "BaseType.h"
#include <memory.h>
#include <fstream>
#include <vector>

class Clothoid
{
public:
	Clothoid()
	{
		_init = false;
	}

	Clothoid(const double& x0, const double &y0, const double& theta0,
		const double& x1, const double &y1, const double& theta1);

	void SetPoint(const double& x0, const double &y0, const double& theta0,
		const double& x1, const double &y1, const double& theta1);
	~Clothoid()
	{

	}
	void BuildClothoid(double tol = 0.05);
	void PointsOnClothoid(std::vector<RoadPoint>& XY, int npts);

protected:
	void FresnelCS(double *FresnelC, double *FresnelS, double y);
	void intXY(double *X, double *Y, int nk, double a, double b, double c);
	void FresnelCSk(int nk, double t, double *FresnelC, double *FresnelS);
	void evalXYaLarge(double *X, double *Y, int nk, double a, double b);
	double S(double mu, double nu, double b);
	void evalXYazero(double*X, double*Y, int nk, double b);
	void evalXYaSmall(double *X, double *Y, int nk, double a, double b, double p);
	void findA(double &A, double Aguess, double delta, double phi0, double tol);
	double guessA(double phi0, double phi1);

private:
	double _k;//斜率
	double _dk;//斜率变化率
	double _L;//曲线长度
	//起点
	double _x0;
	double _y0;
	double _theta0;
	//终点
	double _x1;
	double _y1;
	double _theta1;

	bool _init;
};

#endif // !_CLOTHOID_PATH_GENERATE
