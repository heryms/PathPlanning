#include "Topology.h"
#include <cmath>
using std::vector;

//两点距离的平方
double Topology::Distance2(RoadPoint m1, RoadPoint m2)
{
	return (m1.x - m2.x)*(m1.x - m2.x) + (m1.y - m2.y)*(m1.y - m2.y);
}

//三点余弦
double Topology::cosangle(RoadPoint m1, RoadPoint m2, RoadPoint m3)
{
	return (m2.x - m1.x)*(m2.x - m3.x) + (m2.y - m1.y)*(m2.y - m3.y);
}

double Topology::multiply(RoadPoint sp, RoadPoint ep, RoadPoint op)
{
	return((sp.x - op.x)*(ep.y - op.y) - (ep.x - op.x)*(sp.y - op.y));
}
//符号函数
int Topology::sign(double m)
{
	if (m > 0)
		return 1;
	if (m < 0)
		return -1;

	return 0;
}
//判断线段与线段是否相交
bool Topology::intersect(LINESEG u, LINESEG v)
{
	return((std::max(u.s.x, u.e.x) >= std::min(v.s.x, v.e.x)) &&                     //排斥实验 
		(std::max(v.s.x, v.e.x) >= std::min(u.s.x, u.e.x)) &&
		(std::max(u.s.y, u.e.y) >= std::min(v.s.y, v.e.y)) &&
		(std::max(v.s.y, v.e.y) >= std::min(u.s.y, u.e.y)) &&
		(multiply(v.s, u.e, u.s)*multiply(u.e, v.e, u.s) >= 0) &&         //跨立实验 
		(multiply(u.s, v.e, v.s)*multiply(v.e, u.e, v.s) >= 0));
}

//点与多边形关系判断
bool Topology::InsideConvexPolygon(int vcount, RoadPoint polygon[], RoadPoint q)
{
	return false;
}
//判断两条曲线是否相交，其中GPSPtNum为路网点扩充的数量或gps点的数量
bool Topology::CurveInsects(RoadPoint Up[], RoadPoint Down[], LINESEG u, int GPSPtNum)
{

	bool flag_up, flag_down;
	flag_down = flag_up = 0;

	// TODO:(HERYMS) fileLength REMAIN TO SOLVED
	for (int i = 0; i < GPSPtNum - 1; i++)
	{
		LINESEG l;
		l.s = Up[i];
		l.e = Up[i + 1];
		if (intersect(l, u) == 1)
		{
			flag_up = 1;
			break;
		}
	}

	for (int i = 0; i < GPSPtNum - 1; i++)
	{
		LINESEG l;
		l.s = Down[i];
		l.e = Down[i + 1];
		if (intersect(l, u) == 1)
		{
			flag_down = 1;
			break;
		}
	}
	if (flag_down == 0 && flag_up == 0)
		return 0;
	else return 1;
	//return 0;
}

//获取线段交点
RoadPoint Topology::getIntersectPoint(RoadPoint u1, RoadPoint u2, RoadPoint v1, RoadPoint v2){
	RoadPoint ret = u1;
	double t = ((u1.x - v1.x)*(v1.y - v2.y) - (u1.y - v1.y)*(v1.x - v2.x))
		/ ((u1.x - u2.x)*(v1.y - v2.y) - (u1.y - u2.y)*(v1.x - v2.x));
	ret.x += (u2.x - u1.x)*t;
	ret.y += (u2.y - u1.y)*t;
	return ret;
}

//计算点（x,y）到线段的距离
double Topology::PointToSegDist(double x, double y, double x1, double y1, double x2, double y2)
{
	double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	if (cross <= 0) return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));

	double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	if (cross >= d2) return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));

	double r = cross / d2;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;
	return sqrt((x - px) * (x - px) + (py - y1) * (py - y1));
}

//已知直线上两点，求直线方程ax+by+c=0(a >= 0)
void  Topology::makeline(RoadPoint p1, RoadPoint p2, double &a, double &b, double &c)
{
	int sign = 1;
	a = p2.y - p1.y;
	if (a < 0)
	{
		sign = -1;
		a = sign*(a);
	}
	b = sign*(p1.x - p2.x);
	c = sign*(p1.y*p2.x - p1.x*p2.y);
}
//判断线段和直线是否相交
bool  Topology::Line_Seg_Intersect(double x1, double y1, double x2, double y2, double a, double b, double c)
{
	if ((x1*a + y1*b + c)*(x2*a + y2*b + c) <= 0)
	{
		return true;
	}
	else return false;
}

bool Topology::LineIntersection(double a1, double b1, double c1, double a2, double b2, double c2, RoadPoint &p)
{
	double d = a1*b2 - a2*b1;
	if (d == 0)
	{
		return false;
	}
	p.x = (c2*b1 - c1*b2) / d;
	p.y = (a2*c1 - a1*c2) / d;
	return true;
}

//求三角形角平分线和对边的交点
bool Topology::Angle_Bisector_Intersect(RoadPoint p1, RoadPoint p2, RoadPoint p3, RoadPoint &p)
{
	double A1, B1, C1, A2, B2, C2, A3, B3, C3, A4, B4, C4, A5, B5, C5;
	makeline(p1, p2, A1, B1, C1);
	makeline(p1, p3, A2, B2, C2);
	makeline(p2, p3, A3, B3, C3);

	double Q1, Q2;
	Q1 = sqrt(A1*A1 + B1*B1);
	Q2 = sqrt(A2*A2 + B2*B2);
	A4 = A1*Q2 - A2*Q1;
	B4 = B1*Q2 - B2*Q1;
	C4 = C1*Q2 - C2*Q1;

	A5 = A1*Q2 + A2*Q1;
	B5 = B1*Q2 + B2*Q1;
	C5 = C1*Q2 + C2*Q1;

	if (Line_Seg_Intersect(p2.x, p2.y, p3.x, p3.y, A4, B4, C4))
	{
		LineIntersection(A3, B3, C3, A4, B4, C4, p);
	}
	else
	{
		LineIntersection(A3, B3, C3, A5, B5, C5, p);
	}

	return true;
}
bool Topology::check_velogrid_rdPt_intersected(VeloGrid_t& veloGrids, std::vector<RoadPoint>& rdPt){
	assert(rdPt.size() >= 0);
	int num_pt = rdPt.size();
	for (int i = 0; i < num_pt; i++)
	{
		// may be a bug can be saver
		for (int j = -CAR_WIDTH; j <= CAR_WIDTH; j++)
		{
			if ((int)(rdPt[i].x + 0.5 + j) >= 0 && ((int)(rdPt[i].x + 0.5 + j)) <= MAP_WIDTH - 1)
			{
				int index = MAP_WIDTH*(int)(rdPt[i].y + 0.5) + (int)(rdPt[i].x + 0.5) + j;
				if (veloGrids.velo_grid[index]!=0){//&&veloGrids.velo_grid[index]!=110) {
					return false;
				}
			}
		}


	}
	return true;
}

Eigen::MatrixXd Topology::rotate(double theta, Eigen::MatrixXd in){
	Eigen::MatrixXd A(2, 2);
	A(0, 0) = cos(theta);
	A(0, 1) = -sin(theta);
	A(1, 0) = sin(theta);
	A(1, 1) = cos(theta);

	return A*in;
}

//线性方程组求解
vector<double> Topology::lufact(vector<vector<double>> A, vector<double> B) {
	typedef unsigned int uint;
	uint N = A.size();
	if (N == 0) {
		throw "empty coefficient";
		//return vector<double>(0);
	}
	if (B.size() != N) {
		throw "not a linear system of equations";
	}
	vector<uint> R;
	for (uint i = 0; i < N; i++) {
		R.push_back(i);
	}
	for (uint k = 0; k < N; k++) {
		double max = abs(A[k][k]);
		uint maxIndex = k;
		for (uint i = k + 1; i < N; i++) {
			if (abs(A[i][k]) > max) {
				max = abs(A[i][k]);
				maxIndex = i;
			}
		}
		if (max == 0) {
			throw "singular matrix";
			//return vector<double>(0);
		}
		uint tk = R[k];
		R[k] = R[maxIndex];
		R[maxIndex] = tk;
		for (uint i = 0; i < N; i++) {
			double tki = A[k][i];
			A[k][i] = A[maxIndex][i];
			A[maxIndex][i] = tki;
		}
		for (uint i = k + 1; i < N; i++) {
			A[i][k] /= A[k][k];
			for (uint j = k + 1; j < N; j++) {
				A[i][j] -= A[i][k] * A[k][j];
			}
		}
	}
	vector<double> Y(N, 0);
	Y[0] = B[R[0]];
	for (uint k = 1; k < N; k++) {
		vector<double> TA = A[k];
		double sum = 0;
		for (uint i = 0; i < k; i++) {
			sum += TA[i] * Y[i];
		}
		Y[k] = B[R[k]] - sum;
	}
	vector<double> X(N);
	X[N - 1] = Y[N - 1] / A[N - 1][N - 1];
	for (int k = N - 2; k >= 0; k--) {
		vector<double> TA = A[k];
		double sum = 0;
		for (uint i = k + 1; i < N; i++) {
			sum += TA[i] * X[i];
		}
		X[k] = (Y[k] - sum) / A[k][k];
	}
	return X;
}

Eigen::MatrixX4d Topology::CubicSpline(std::vector<double> X, std::vector<double> Y, double dds0, double ddsn)
{
	auto Xiter = X.rbegin();
	auto Yiter = Y.rbegin();
	int N = X.size() - 1;
	std::vector<double> h(N);
	std::vector<double> d(N);
	std::vector<double> u(N - 1);
	auto hiter = h.rbegin();
	auto diter = d.rbegin();
	auto uiter = u.rbegin();
	for (; Xiter != X.rend() - 1; hiter++, diter++) {
		*hiter = *Xiter - *(++Xiter);
		*diter = (*Yiter - *(++Yiter)) / *hiter;
	}
	for (diter = d.rbegin(); diter != d.rend() - 1; uiter++) {
		*uiter = (*diter - *(++diter)) * 6;
	}
	vector<vector<double>> A(N - 1, vector<double>(N - 1));
	vector<double> B(N - 1);
	A[0][0] = 2 * (h[0] + h[1]);
	A[0][1] = h[1];
	B[0] = u[0] - h[0] * dds0;
	A[N - 2][ N - 3] = h[N - 2];
	A[N - 2][ N - 2] = 2 * (h[N - 3] + h[N - 2]);
	B[N - 2] = u[N - 2] - h[N - 2] * ddsn;
	for (int k = 2; k <= N - 2; k++) {
		A[k - 1][k - 2] = h[k - 1];
		A[k - 1][k - 1] = 2 * (h[k - 1] + h[k]);
		A[k - 1][k] = h[k];
		B[k - 1] = u[k - 1];
	}
	vector<double> m=Topology::lufact(A, B);
	m.insert(m.begin(), dds0);
	m.push_back(ddsn);
	Eigen::MatrixX4d s;
	s.resize(N, 4);
	for (int i = 0; i < N; i++) {
		s(i,0) = Y[i];
		s(i, 1) = d[i] - h[i] * (2 * m[i] + m[i + 1]) / 6;
		s(i, 2) = m[i] / 2;
		s(i, 3) = (m[i + 1] - m[i]) / (6 * h[i]);
	}
	return s;
}

void Topology::Rotate_To_Decare(double theta, double x, double y, double& x_out, double& y_out)
{
	x_out = y * cos(theta) - x * sin(theta);
	y_out = y * sin(theta) + x * cos(theta);

}
void Topology::Rotate(double theta, double x, double y, double& x_out, double& y_out)
{
	x_out = y * cos(theta) - x * sin(theta);
	y_out = y * sin(theta) + x * cos(theta);

}
void Topology::Rotate_To_Guassian(double theta, double x, double y, double& x_out, double& y_out){
	// x, y should be decarle coordinate
	// theta should be same as above
	y_out = x * cos(theta) + y * sin(theta);
	x_out = -x * sin(theta) + y * cos(theta);
}
double Topology::toAngle(double dx, double dy){

	// result -pi pi
	if (dy >=0 && dx >=0)
	{
		return atan(dy / dx);
	}
	if (dy>=0 && dx<=0)
	{
		return atan(dy / dx) + PI;
	}
	if (dy<=0 && dx <= 0)
	{
		return atan(dy / dx) - PI;
	}
	if (dy <= 0 && dx >= 0){
		return atan(dy / dx);
	}
}