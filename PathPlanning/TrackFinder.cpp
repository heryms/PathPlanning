#include "TrackFinder.h"
#include "Topology.h"
#include "LocalCarStatus.h"


TrackFinder::TrackFinder()
{
}


TrackFinder::~TrackFinder()
{
}

int TrackFinder::FindPointIndex(std::vector<RoadPoint>& path, RoadPoint curX)
{
	if (path.size() <= 1) {
		return -1;
	}
	std::vector<int> point;
	double dis = 1;
	while (point.empty())
	{
		for (int i = 0; i < path.size(); i++) {
			if (Topology::Distance2(path[i], curX) < dis*dis) {
				if (cos(path[i].angle-curX.angle) > 0)
				{
					point.push_back(i);
				}
			}
		}
		dis += 1;
		if (dis > 10) {
			return -1;
		}
	}
	int minIndex = point[0];
	double min = Topology::Distance2(path[0], curX);
	for (int index : point) {
		double t = Topology::Distance2(path[index], curX);
		if (t < min) {
			min = t;
			minIndex = index;
		}
	}
	return minIndex;
}

int TrackFinder::AnchorPoint(std::vector<RoadPoint>& path, RoadPoint curX, int curIndex, double distance)
{
	if (curIndex < 0) {
		curIndex = FindPointIndex(path, curX);
	}
	if (curIndex < 0) {
		return  -1;
	}
	if (curIndex >= path.size()) {
		return -1;
	}
	double dis = sqrt(Topology::Distance2(path[curIndex], curX));
	for (int i = curIndex + 1; i < path.size(); i++) {
		dis += sqrt(Topology::Distance2(path[i], path[i - 1]));
		if (distance < dis) {
			return i;
		}
	}
	return path.size() - 1;
}

int TrackFinder::AnchorPointStraight(std::vector<RoadPoint>& path, RoadPoint curX, int curIndex, double distance)
{
	if (curIndex < 0) {
		curIndex = FindPointIndex(path, curX);
	}
	if (curIndex < 0) {
		return  -1;
	}
	if (curIndex >= path.size()) {
		return -1;
	}
	double dis = sqrt(Topology::Distance2(path[curIndex], curX));
	for (int i = curIndex + 1; i < path.size(); i++) {
		dis = sqrt(Topology::Distance2(path[i], curX));
		if (distance < dis) {
			return i;
		}
	}
	return path.size() - 1;
}

bool TrackFinder::InCurve(bool isInCurve, std::vector<RoadPoint>& path, RoadPoint curX, int curIndex)
{
	double anchorDis = 20;
	int anchorIndex = AnchorPoint(path, curX, curIndex, anchorDis);
	if(anchorIndex<0){
		return isInCurve;
	}
	if (Topology::Distance2(path[anchorIndex], curX)<(isInCurve ? 0.999 : 0.97)*anchorDis*anchorDis) {
		return true;
	}
	else {
		return false;
	}
}
double  TrackFinder::calulate_radios_err(double x, double y, double radios)
{
	return (x*x + y*y - radios*radios);
}
RoadPoint TrackFinder::Newton_divide(RoadPoint p0, RoadPoint p1, float radious)
{
	//计算误差
	double x0 = p0.x;
	double y0 = p0.y;
	double x1 = p1.x;
	double y1 = p1.y;
	double midx = (x0 + x1) / 2;
	double midy = (y0 + y1) / 2;
	RoadPoint midpoint;
	midpoint.x = midx;
	midpoint.y = midy;
	double r0 = calulate_radios_err(x0, y0, radious);
	double r1 = calulate_radios_err(x1, y1, radious);
	double r2 = calulate_radios_err(midx, midy, radious);
	if (fabs(r0) <= 0.01)
	{
		return p0;

	}
	if (fabs(r1) <= 0.01)
	{
		return p1;
	}
	if (fabs(r2) <= 0.01)
	{
		return midpoint;
	}
	if (r0*r2 <= 0)
	{
		return Newton_divide(p0, midpoint, radious);
	}
	else if (r1*r2 <= 0)
	{
		return Newton_divide(midpoint, p1, radious);
	}
}

//改进版纯追随控制算法，坐标系参考为车辆后轮转轴中间
bool TrackFinder::FindPursuitPoint(std::vector<RoadPoint>& refPath, RoadPoint curX, RoadPoint &refX, float Speed)
{
	if (refPath.empty()) {
		return false;
	}
	int minIndex = 0;
	double minDistance;
	{
		double dx = refPath[0].x;
		double dy = refPath[0].y;
		minDistance = dx*dx + dy*dy;
	}
	for (int i = 1; i < refPath.size(); i++) {
		double dx = refPath[i].x;
		double dy = refPath[i].y;
		double dis = dx*dx + dy*dy;
		if (dis < minDistance) {
			minDistance = dis;
			minIndex = i;
		}
	}
	double radius = 5.5;//预瞄半径+
	float Kla = 2.25; //比例系数
	float Vchange = 2.25;  //速度阈值 单位为m/s
	if (Speed <= Vchange)
	{
		//radius = Lmin;
		//+delD*Kct;
	}
	else
	{
		radius = Speed*Kla;
		//+delD*Kct;
	}
	bool isFind = false;
	double towardX = sin(PI/2-curX.angle);
	double towardY = cos(PI/2-curX.angle);
	for (int i = 1; i < refPath.size(); i++) {
		double delGaux0 = refPath[i - 1].x;
		double delGauy0 = refPath[i - 1].y;
		double delGaux1 = refPath[i].x;
		double delGauy1 = refPath[i].y;
		double r0 = delGaux0*delGaux0 + delGauy0*delGauy0 - radius*radius;
		double r1 = delGaux1*delGaux1 + delGauy1*delGauy1 - radius*radius;
		if (fabs(r0) < 0.01) {
			if (towardX*delGaux0 + towardY*delGauy0 > 0) {
				refX = refPath[i - 1];
				isFind = true;
				break;
			}
		}

		if (fabs(r1) < 0.01) {
			if (towardX*delGaux1 + towardY*delGauy1 > 0) {
				refX = refPath[i];
				isFind = true;
				break;
			}
		}

		if (r0*r1 < 0) {
			RoadPoint tag0 = Newton_divide(refPath[i - 1], refPath[i], radius);
			if (towardX*tag0.x + towardY*tag0.y > 0) {
				refX = tag0;
				isFind = true;
				break;
			}
		}
	}
	if (isFind) {
		//refX.x += curX.x;
		//refX.y += curX.y;
	}
	return isFind;
}

short TrackFinder::MiddlePiont_Shan(double firMidPoint_x, double firMidPoint_y)
{
	static float a_theta = 0;
	static int n = 1;
	static float angle_previous = 0;
	double a;					//a:车辆前轮转角
	double delt_b;				//delt_b:车辆当前方向和预瞄方向的夹角  预瞄方向：车两后轮中心和预瞄点的连线
	const double L = LocalCarStatus::GetInstance().GetL();		//常量 L:车前后车轮轴距.暂定为2.5米
	double Lf;					//Lforward，后轮中心与预瞄点的距离
	const double delt_y = 0;		//常量 车前杠中点和两后轮中心的距离，y方向上
	double c;					//方向盘转角
	double kp = 1;				//转角增益
	double error = 0;
	delt_b = atan(firMidPoint_x / (firMidPoint_y));
	Lf = sqrt((firMidPoint_x*firMidPoint_x) + (firMidPoint_y)*(firMidPoint_y));
	a = atan(2 * L*sin(delt_b) / Lf);
	a_theta = a*180.0* LocalCarStatus::GetInstance().GetSteerRatio()/ PI;
	short changeangle;
	changeangle = a_theta;

	return changeangle;
}


