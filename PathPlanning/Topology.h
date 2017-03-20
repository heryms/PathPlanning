#ifndef _TOPOLOGY_MATH_METHOD
#define _TOPOLOGY_MATH_METHOD
#include "BaseType.h"
#include "Variables.h"
#include <cmath>
#include <algorithm>
int fileLength = 100;
bool InsideConvexPolygon(int vcount, RoadPoint polygon[], RoadPoint q);//点与多边形关系判断
bool CurveInsects(RoadPoint Up[], RoadPoint Down[], LINESEG u);//
bool intersect(LINESEG u, LINESEG v);//直线相交算法
double Distance(RoadPoint m1, RoadPoint m2);  //欧几里得距离
double cosangle(RoadPoint m1, RoadPoint m2, RoadPoint m3); //三点余弦
double multiply(RoadPoint sp, RoadPoint ep, RoadPoint op);
int sign(double m);
RoadPoint getIntersectPoint(RoadPoint u1, RoadPoint u2, RoadPoint v1, RoadPoint v2);//求两条线段的交点
double PointToSegDist(double x, double y, double x1, double y1, double x2, double y2);//求点（x,y）到线段的距离
void makeline(RoadPoint p1, RoadPoint p2, double &a, double &b, double &c);//已知直线上两点，求直线方程ax+by+c=0的因子a,b,c
bool Line_Seg_Intersect(double x1, double y1, double x2, double y2, double a, double b, double c);//判断线段和直线是否相交
bool LineIntersection(double a1, double b1, double c1, double a2, double b2, double c2, RoadPoint &p);//求直线和直线的交点
bool Angle_Bisector_Intersect(RoadPoint p1, RoadPoint p2, RoadPoint p3, RoadPoint &p);//求三角形角平分线和对边的交点(角是p1所在的角）
#endif