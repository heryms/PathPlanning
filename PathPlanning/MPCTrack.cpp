#include "MPCTrack.h"
#include "TrackFinder.h"
#include "LocalCarStatus.h"
#include "Topology.h"
#include "CarControl.h"
#include "ForMPC\QuadProg++.hh"
MPCTrack::MPCTrack()
{
}


MPCTrack::~MPCTrack()
{
}

void MPCTrack::SetPath(std::vector<RoadPoint>& path)
{
	BaseTrack::SetPath(path);
	start = end = std::chrono::high_resolution_clock::now();
	lastX = RoadPoint::toRoadPoint(LocalCarStatus::GetInstance().GetPosition());
}

void MPCTrack::Track()
{
	if (path.size() < 10) {
		return;
	}
	end = std::chrono::high_resolution_clock::now();
	microseconds =
		std::chrono::duration_cast<std::chrono::microseconds>
		(end - start);
	CarInfo info;
	RoadPoint curX = RoadPoint::toRoadPoint(LocalCarStatus::GetInstance().GetPosition());
	int curIndex = TrackFinder::FindPointIndex(path, curX);
	do {
		if (curIndex < 0) {
			info.gear = D;
			info.speed = 0;
			info.state = E_STOP;
			info.steerAngle = LocalCarStatus::GetInstance().GetSteerAngle();
			break;
		}
		inCurve = TrackFinder::InCurve(inCurve, path, curX, curIndex);
		if (inCurve) {
			info.speed = refSpeedCurve;
		}
		else {
			info.speed = refSpeedStraight;
		}
		info.gear = D;
		info.state = START;
		double curSpeed = LocalCarStatus::GetInstance().GetSpeed();
		for (int i = 0; i < simPeriod; i++) {
			refXs[i] = path[TrackFinder::AnchorPoint
			(path, curX, curIndex,
				i * fmax(curSpeed - 1.2,
					fmin(info.speed, curSpeed + 1)) / 3.6 * microseconds.count() / 1000000.0)];
			refXs[i].x *= 0.2;
			refXs[i].y *= 0.2;
		}
		curX.x *= 0.2;
		curX.y *= 0.2;
		RealTrack(info, curSpeed, LocalCarStatus::GetInstance().GetSteerAngle(), microseconds.count() / 1000000.0,curX);
		lastX = RoadPoint::toRoadPoint(LocalCarStatus::GetInstance().GetPosition());
		start = std::chrono::high_resolution_clock::now();
	} while (false);
	CarControl::GetInstance().SendCommand(info);
}

void MPCTrack::RealTrack(CarInfo& info, double curSpeed, double curSteerAngle, double Tctrl, RoadPoint curX) {
	double v = curSpeed / 3.6;//m/s
	double theta = curX.angle;//by axis x , rad
	double delta = RadAngle::Normalize(curSteerAngle / LocalCarStatus::GetInstance().GetSteerRatio()*PI / 180.0);//rad
	double phi = theta;
	double Tsample = Tctrl;
	int Nq = Nx;
	double L = LocalCarStatus::GetInstance().GetL();
	//CMatrix<double> Cm(Nq, Nx);
	CMatrix<double> Cm = CMatrix<double>::Identity(Nq);
	Cm(2, 2) = 0.01;
	CMatrix<double> Am({
		{ 1,0,0 }
		,{ 0,1,0 }
		,{ -v*sin(phi)*Tsample,v*cos(phi)*Tsample,1 }
	});
	CMatrix<double> Bm({
		{ cos(phi),sin(phi),-tan(delta) / L },
		{ 0,0,-v / (L*pow(cos(delta),2)) }
	});
	Bm *= Tsample;
	int Nc = Nu;
	int Tp = simPeriod;
	int Tc = simPeriod;// / 2;// / 3;// 20 > simPeriod ? simPeriod : 20;
	CMatrix<double> A(Nx + Nq, Nx + Nq);
	A.Assign({ 0,0 }, Am);
	A.Assign({ 0,Nx }, CMatrix<double>::Zero(Nx, Nq));
	A.Assign({ Nx,0 }, Cm*Am);
	A.Assign({ Nx,Nx }, CMatrix<double>::Identity(Nq));
	CMatrix<double> C(Nq, Nx + Nq);
	C.Assign({ 0,0 }, CMatrix<double>::Zero(Nq, Nx));
	C.Assign({ 0,Nx }, CMatrix<double>::Identity(Nq));
	CMatrix<double> CA = C*A;
	CMatrix<double> F(Tp*Nq, Nx + Nq);
	F.Assign({ 0,0 }, CA);
	for (int i = 1; i < Tp; i++) {
		CA *= A;
		F.Assign({ i*Nq,0 }, CA);
	}
	CMatrix<double> B(Nx + Nq, Nc);
	B.Assign({ 0,0 }, Bm);
	B.Assign({ Nx,0 }, Cm*Bm);
	CMatrix<double> PHI = CMatrix<double>::Zero(Nq*Tp, Tc*Nc);
	CMatrix<double> CAB = C*A*B;
	for (int i = 0; i < Tc; i++) {
		PHI.Assign({ i*Nq,i*Nc }, CAB);
	}
	for (int j = 1; j < Tp; j++) {
		CAB = F.Copy({ Nq*(j - 1),0 }, { Nq*j - 1,(Nx + Nq) - 1 })*B;
		for (int i = j; i < j + Tc && i < Tp; i++) {
			PHI.Assign({ i*Nq,(i - j)*Nc }, CAB);
		}
	}
	double dX = curX.x - lastX.x, dY = curX.y - lastX.y;
	RadAngle dO = curX.angle - lastX.angle;
	CMatrix<double> xki({
		dX,dY,dO,curX.x,curX.y,curX.angle
	});
	CMatrix<double> Rs(Tp*Nq, 1);
	for (int i = 0; i < Rs.Size(); i += Nq) {
		Rs[i] = refXs[i / Nq].x;
		Rs[i + 1] = refXs[i / Nq].y;
		Rs[i + 2] = refXs[i / Nq].angle;
	}
	CMatrix<double> R = CMatrix<double>::Identity(Tc*Nc);
	for (int i = 0; i < R.ColumnCount(); i += 2) {
		R(i, i) = 1;
		R(i + 1, i + 1) = 1000;
	}
	CMatrix<double> E = (PHI.Transpose()*PHI + R) * 2;
	CMatrix<double> dRs = Rs - F*xki;
	for (int i = 0; i < dRs.Size(); i += Nq) {
		dRs[i + 2] = (RadAngle::Normalize(dRs[i + 2]));
	}
	CMatrix<double> f = (PHI.Transpose()*dRs)*-2;

	double dvmin = fmax(-v, -4 * Tsample);
	double dv = info.speed / 3.6 - v;
	double dvmax = fmin(dv > 0 ? dv : fmax(-4 * Tsample, dv), Tsample);// :fmax(dv, -4 * Tsample);
	double ddeltamin = fmax(-0.52 - delta, -0.26*Tsample);
	double ddeltamax = fmin(0.52 - delta, 0.26*Tsample);


	CMatrix<double> Acoff = CMatrix<double>::Zero(Nc * 2, Nc * Tc);
	Acoff(0, 0) = 1;
	Acoff(1, 1) = 1;
	Acoff(2, 0) = -1;
	Acoff(3, 1) = -1;
	CMatrix<double> bcoff(Nc * 2, 1, { -dvmin,-ddeltamin,dvmax,ddeltamax });

	CMatrix<double> X = QuadProg(E, f, Acoff, bcoff, CMatrix<double>(0, Nc*Tc), CMatrix<double>(0, 1));//, lb, ub);

	info.speed = (v + X(0, 0))*3.6;
	info.steerAngle = (delta + X(1, 0))*LocalCarStatus::GetInstance().GetSteerRatio()*180.0 / PI;
}

inline Matrix<double> MPCTrack::ToMat(CMatrix<double>& H)
{
	Matrix<double> G;
	G.resize(H.RowCount(), H.ColumnCount());
	for (unsigned int i = 0; i < H.RowCount(); i++) {
		for (unsigned int j = 0; j < H.ColumnCount(); j++) {
			G[i][j] = H(i, j);
		}
	}
	return G;
}

inline Vector<double> MPCTrack::ToVec(CMatrix<double>& f)
{
	Vector<double> g;
	g.resize(f.Size());
	for (unsigned int i = 0; i < f.Size(); i++) {
		g[i] = f[i];
	}
	return g;
}

inline CMatrix<double> MPCTrack::FromVec(Vector<double>& x)
{
	CMatrix<double> d(x.size(), 1);
	for (unsigned int i = 0; i < x.size(); i++) {
		d[i] = x[i];
	}
	return d;
}

CMatrix<double> MPCTrack::QuadProg(CMatrix<double>& H, CMatrix<double>& f, CMatrix<double>& A, CMatrix<double>& b, CMatrix<double>& Aeq, CMatrix<double>& beq)//, CMatrix<double> lb, CMatrix<double> ub)
{
	Matrix<double> G = ToMat(H.Transpose());
	Vector<double> g0 = ToVec(f);
	Matrix<double> CE = ToMat(Aeq.Transpose());
	Vector<double> Ce0 = ToVec(beq);
	Matrix<double> CI = ToMat(A.Transpose());
	Vector<double> Ci0 = ToVec(b);
	Vector<double> x;
	x.resize(H.RowCount());
	solve_quadprog(G, g0, CE, Ce0, CI, Ci0, x);
	return FromVec(x);
}
