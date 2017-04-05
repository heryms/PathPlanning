#pragma once
#ifndef _PATH_PLANNING_MPC_TRACK_H
#define _PATH_PLANNING_MPC_TRACK_H
#include "BaseTrack.h"
#include <chrono>
#include "CarControl.h"
#include "ForMPC\Array.hh"
#include "ForMPC\MMatrix.h"
using MMatrix::CMatrix;
class MPCTrack : public BaseTrack
{
private:
	int Nx = 3;
	int Nu = 2;
	std::chrono::high_resolution_clock::time_point start, end;
	std::chrono::microseconds microseconds;
	const static int simPeriod = 60;
	RoadPoint refXs[simPeriod];
	void RealTrack(CarInfo& info, double curSpeed, double curSteerAngle, double Tctrl, RoadPoint curX);
	inline Matrix<double> ToMat(CMatrix<double>& H);
	inline Vector<double> ToVec(CMatrix<double>& f);
	inline CMatrix<double> FromVec(Vector<double>& x);
	CMatrix<double> QuadProg(CMatrix<double>& H, CMatrix<double>& f, CMatrix<double>& A, CMatrix<double>& b, CMatrix<double>& Aeq, CMatrix<double>& beq);// , CMatrix<double> lb, CMatrix<double> ub);
	RoadPoint lastX;
public:
	MPCTrack();
	~MPCTrack();
	void SetPath(std::vector<RoadPoint>& path);
	void Track();
};


#endif // !_PATH_PLANNING_PURE_TRACK_H