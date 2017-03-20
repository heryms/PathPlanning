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
void FresnelCS(double *FresnelC, double *FresnelS, double y);
void intXY(double *X, double *Y, int nk, double a, double b, double c);
void FresnelCSk(int nk, double t, double *FresnelC, double *FresnelS);
void evalXYaLarge(double *X, double *Y, int nk, double a, double b);
double S(double mu, double nu, double b);
void evalXYazero(double*X, double*Y, int nk, double b);
void evalXYaSmall(double *X, double *Y, int nk, double a, double b, double p);
void buildClothoid(double &k, double &dk, double &L, double x0, double y0, double theta0, double x1, double y1, double theta1, double tol);
double normalizeAngle(double phi_in);
void findA(double &A, double Aguess, double delta, double phi0, double tol);
double guessA(double phi0, double phi1);
void pointsOnClothoid(RoadPoint XY[], double x0, double y0, double theta0, double kappa, double dkappa, double L, int npts);
#endif // !_CLOTHOID_PATH_GENERATE
