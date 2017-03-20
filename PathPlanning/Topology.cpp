#include "Topology.h"
double Distance(RoadPoint m1, RoadPoint m2)
{
	return (m1.x - m2.x)*(m1.x - m2.x) + (m1.y - m2.y)*(m1.y - m2.y);
}
double cosangle(RoadPoint m1, RoadPoint m2, RoadPoint m3)
{
	return (m2.x - m1.x)*(m2.x - m3.x) + (m2.y - m1.y)*(m2.y - m3.y);
}
double multiply(RoadPoint sp, RoadPoint ep, RoadPoint op)
{
	return((sp.x - op.x)*(ep.y - op.y) - (ep.x - op.x)*(sp.y - op.y));
}
//判断线段与线段是否相交
bool intersect(LINESEG u, LINESEG v)
{
	return((std::max(u.s.x, u.e.x) >= std::min(v.s.x, v.e.x)) &&                     //排斥实验 
		(std::max(v.s.x, v.e.x) >= std::min(u.s.x, u.e.x)) &&
		(std::max(u.s.y, u.e.y) >= std::min(v.s.y, v.e.y)) &&
		(std::max(v.s.y, v.e.y) >= std::min(u.s.y, u.e.y)) &&
		(multiply(v.s, u.e, u.s)*multiply(u.e, v.e, u.s) >= 0) &&         //跨立实验 
		(multiply(u.s, v.e, v.s)*multiply(v.e, u.e, v.s) >= 0));
}
bool CurveInsects(RoadPoint Up[], RoadPoint Down[], LINESEG u)
{

	bool flag_up, flag_down;
	flag_down = flag_up = 0;

	// TODO:(HERYMS) fileLength REMAIN TO SOLVED
	for (int i = 0; i < fileLength - 1; i++)
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

	for (int i = 0; i < fileLength - 1; i++)
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
RoadPoint getIntersectPoint(RoadPoint u1, RoadPoint u2, RoadPoint v1, RoadPoint v2){
	RoadPoint ret = u1;
	double t = ((u1.x - v1.x)*(v1.y - v2.y) - (u1.y - v1.y)*(v1.x - v2.x))
		/ ((u1.x - u2.x)*(v1.y - v2.y) - (u1.y - u2.y)*(v1.x - v2.x));
	ret.x += (u2.x - u1.x)*t;
	ret.y += (u2.y - u1.y)*t;
	return ret;
}

//计算点（x,y）到线段的距离
double PointToSegDist(double x, double y, double x1, double y1, double x2, double y2)
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
void  makeline(RoadPoint p1, RoadPoint p2, double &a, double &b, double &c)
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
bool  Line_Seg_Intersect(double x1, double y1, double x2, double y2, double a, double b, double c)
{
	if ((x1*a + y1*b + c)*(x2*a + y2*b + c) <= 0)
	{
		return true;
	}
	else return false;
}

bool LineIntersection(double a1, double b1, double c1, double a2, double b2, double c2, RoadPoint &p)
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
bool Angle_Bisector_Intersect(RoadPoint p1, RoadPoint p2, RoadPoint p3, RoadPoint &p)
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


