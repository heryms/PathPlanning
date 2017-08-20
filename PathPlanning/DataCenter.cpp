#include "DataCenter.h"
#include "lcmtype\LcmSet.h"
#include "LocalCarStatus.h"
#include "Topology.h"
#include "PathGenerateTool.h"
#include <cmath>
#include <iomanip>
#include <utility>
#include <iostream>
#include "CoordTransform.h"

typedef std::unique_lock<std::mutex> QuickLock;

void DataCenter::RefTrajectoryRecvOperation(const ckMapStatus_t * msg, void *)
{
	QuickLock lk(m_lockRefTrajectory);
	m_lcmMsgRefTrajectory = *msg;
	m_waitRefTrajectory.notify_all();
}

DataCenter::DataCenter()
{

}

DataCenter::~DataCenter()
{
	EndAllSensor();
}

DataCenter& DataCenter::GetInstance() {
	static DataCenter dataCenter;
	return dataCenter;
}

void DataCenter::LocationRecvOperation(const Location_t* msg, void*){
	QuickLock lk(m_lockLocation);
	m_lcmMsgLocation = *msg;
	//PosPoint pos;
	//pos.x = m_lcmMsgLocation.gau_pos[1] + 500000;
	//pos.y = m_lcmMsgLocation.gau_pos[0];
	//pos.angle = PI / 2 - m_lcmMsgLocation.orientation[2] * PI / 180;
	//LocalCarStatus::GetInstance().SetCurPosition(pos);
	m_waitLocation.notify_all();
}

void DataCenter::StatusBodyRecvOperation(const StatusBody_t* msg, void*){
	QuickLock lk(m_lockStatusBody);
	m_lcmMsgStatusBody = *msg;
	//LocalCarStatus::GetInstance().SetSteerAngle(msg->wheelAngle);
	//LocalCarStatus::GetInstance().SetSpeed(msg->vehicleSpeed);
	m_waitStatusBody.notify_all();
}

void DataCenter::VeloGridRecvOperation(const VeloGrid_t* msg, void*){
	QuickLock lk(m_lockVeloGrid);
	m_lcmMsgVeloGrid = *msg;	
	m_waitVeloGrid.notify_all();
}

void DataCenter::CurbRecvOperation(const cloudHandler* msg, void*){
	QuickLock lk(m_lockCurb);
	m_lcmMsgCurb = *msg;
	m_waitCurb.notify_all();
}

void DataCenter::MultiLaneRecvOperation(const Map_t* msg, void*){
	QuickLock lk(m_lockMultiLane);
	m_lcmMsgMultiLane = *msg;
	m_waitMultiLane.notify_all();
}

void DataCenter::CamLaneRecvOperation(const Lane_t * msg, void *)
{
	QuickLock lk(m_lockCamLane);
	m_lcmMsgCamLane = *msg;
	m_waitCamLane.notify_all();
}

void DataCenter::SceneRecvOperation(const Scene_t* msg, void *)
{
	QuickLock lk(m_lockScene);
	m_lcmMsgScene = *msg;
	m_waitScene.notify_all();
}

void DataCenter::StartAllSensor(){
	StartLocation();
	StartStatusBody();
	StartVeloGrid();
	StartCurb();
	StartRefTrajectory();
	StartMultiLane();
	StartCamLane();
	StartScene();
	//m_lcmLocation.initialLcm(LCM_NET_LOCATION, LCM_CHANNEL_LOCATION
	//	, std::bind(&DataCenter::LocationRecvOperation, this,std::placeholders::_1,std::placeholders::_2),this);
	//m_lcmStatusBody.initialLcm(LCM_NET_STATUS_BODY, LCM_CHANNEL_STATUS_BODY
	//	, std::bind(&DataCenter::StatusBodyRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
	//m_lcmVeloGrid.initialLcm(LCM_NET_VELO_GRID, LCM_CHANNEL_VELO_GRID
	//	, std::bind(&DataCenter::VeloGridRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
	//m_lcmCurb.initialLcm(LCM_NET_CLOUD, LCM_CHANNEL_CLOUD
	//	, std::bind(&DataCenter::CurbRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::EndAllSensor(){
	EndLocation();
	EndStatusBody();
	EndVeloGrid();
	EndCurb();
	EndRefTrajectory();
	EndMultiLane();
	EndCamLane();
	EndScene();
	//m_lcmLocation.uninitialLcm();
	//m_lcmStatusBody.uninitialLcm();
	//m_lcmVeloGrid.uninitialLcm();
	//m_lcmCurb.uninitialLcm();
}

void DataCenter::StartLocation()
{
	m_lcmLocation.initialLcm(LCM_NET_LOCATION, LCM_CHANNEL_LOCATION
		, std::bind(&DataCenter::LocationRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::StartStatusBody()
{
	m_lcmStatusBody.initialLcm(LCM_NET_STATUS_BODY, LCM_CHANNEL_STATUS_BODY
		, std::bind(&DataCenter::StatusBodyRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::StartVeloGrid()
{
	m_lcmVeloGrid.initialLcm(LCM_NET_VELO_GRID, LCM_CHANNEL_VELO_GRID
		, std::bind(&DataCenter::VeloGridRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::StartCurb()
{
	m_lcmCurb.initialLcm(LCM_NET_CLOUD, LCM_CHANNEL_CLOUD
		, std::bind(&DataCenter::CurbRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::StartRefTrajectory()
{
	m_lcmRefTrajectory.initialLcm(LCM_NET_REFERENCE_TRAJECTORY, LCM_CHANNEL_REFERENCE_TRAJECTORY,
		std::bind(&DataCenter::RefTrajectoryRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}
void DataCenter::StartMultiLane()
{
	m_lcmMultiLane.initialLcm(LCM_NET_MAP, LCM_CHANNEL_MAP
		, std::bind(&DataCenter::MultiLaneRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::StartCamLane()
{
	m_lcmCamLane.initialLcm(LCM_NET_CAM_LANE, LCM_CHANNEL_CAM_LANE
		, std::bind(&DataCenter::CamLaneRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::StartScene()
{
	m_lcmScene.initialLcm(LCM_NET_SCENE, LCM_CHANNEL_SCENE
		, std::bind(&DataCenter::SceneRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::EndLocation()
{
	m_lcmLocation.uninitialLcm();
}

void DataCenter::EndStatusBody()
{
	m_lcmStatusBody.uninitialLcm();
}

void DataCenter::EndVeloGrid()
{
	m_lcmVeloGrid.uninitialLcm();
}

void DataCenter::EndCurb()
{
	m_lcmCurb.uninitialLcm();
}

void DataCenter::EndRefTrajectory()
{
	m_lcmRefTrajectory.uninitialLcm();
}

void DataCenter::EndMultiLane(){
	m_lcmMultiLane.uninitialLcm();
}

void DataCenter::EndCamLane()
{
	m_lcmCamLane.uninitialLcm();
}

void DataCenter::EndScene()
{
	m_lcmScene.uninitialLcm();
}

PosPoint DataCenter::GetCurPosition(){
	QuickLock lk(m_lockLocation);
	m_curPos.x = m_lcmMsgLocation.gau_pos[1];
	m_curPos.y = m_lcmMsgLocation.gau_pos[0];
	m_curPos.angle = PI / 2 - m_lcmMsgLocation.orientation[2]*PI/180;
	return m_curPos;
}

CarInfo DataCenter::GetCarInfo(){
	QuickLock lk(m_lockStatusBody);
	m_curCarInfo.speed = m_lcmMsgStatusBody.vehicleSpeed;
	m_curCarInfo.steerAngle = m_lcmMsgStatusBody.wheelAngle;
	m_curCarInfo.gear =(EGear)m_lcmMsgStatusBody.gear;
	m_curCarInfo.state =(ERunState) m_lcmMsgStatusBody.controlStatus;
	return m_curCarInfo;
}

VeloGrid_t DataCenter::GetLidarData()
{
	QuickLock lk(m_lockVeloGrid);
	return m_lcmMsgVeloGrid;
}

PosPoint DataCenter::GetRoadEdgePoint(double y, CurbDirection dir)
{
	QuickLock lk(m_lockCurb);
	PosPoint pt;
	pt.y = y;
	double x;
	switch (dir)
	{
	case LEFT:
		if (m_lcmMsgCurb.cloud[0].x)
		{
			x = -(m_lcmMsgCurb.cloud[0].y * y + m_lcmMsgCurb.cloud[0].z) / m_lcmMsgCurb.cloud[0].x;
			pt.x = x;
		}
		else
		{
			x = 0;
			pt.y = -m_lcmMsgCurb.cloud[0].z / m_lcmMsgCurb.cloud[0].y;
			pt.x = x;
		}

		//pt.angle = atan(y / x);
		break;
	case RIGHT:
		if (m_lcmMsgCurb.cloud[3].x)
		{
			x = -(m_lcmMsgCurb.cloud[3].y * y + m_lcmMsgCurb.cloud[3].z) / m_lcmMsgCurb.cloud[3].x;
			pt.x = x;
		}
		else
		{
			x = 0;
			pt.y = -m_lcmMsgCurb.cloud[3].z / m_lcmMsgCurb.cloud[3].y;
		}

		break;
	default:
		break;
	}

	pt.angle = PI + atan(y / x);
	return pt;
}

PCLcloudShow::pointXYZI DataCenter::GetRoadEdgeCoefficient(CurbDirection dir)
{
	QuickLock lk(m_lockCurb);
	PCLcloudShow::pointXYZI coeff;
	switch (dir)
	{
	case LEFT:
		coeff = m_lcmMsgCurb.cloud[0];
		break;
	case RIGHT:
		coeff = m_lcmMsgCurb.cloud[3];
		break;
	default:
		break;
	}
	return PCLcloudShow::pointXYZI();
}

PosPoint DataCenter::GetTargetPoint() {
	return PosPoint();
}
PosPoint DataCenter::GetCurOnTrajectory(){
	QuickLock lk(m_lockRefTrajectory);
	PosPoint pt;
	pt.x = m_lcmMsgRefTrajectory.x[0];
	pt.y = m_lcmMsgRefTrajectory.y[0];
	return pt;
}

Scene_t DataCenter::GetSceneMessage()
{
	return m_lcmMsgScene;
}

std::vector<RoadPoint> DataCenter::GetReferenceTrajectory(RoadPoint &car) {
	QuickLock lk(m_lockRefTrajectory);
	std::vector<RoadPoint> trajectory;
	trajectory.reserve(m_lcmMsgRefTrajectory.num);
	PosPoint curpt = GetCurPosition();
	double angle = curpt.angle;
	//double min = DBL_MAX;
	int minIndex = 0;
	for (int i = 0; i < m_lcmMsgRefTrajectory.num; i++)
	{
		RoadPoint pt;
		double dx = m_lcmMsgRefTrajectory.x[i] - m_lcmMsgRefTrajectory.x[0];
		double dy = m_lcmMsgRefTrajectory.y[i] -  m_lcmMsgRefTrajectory.y[0];
																  // *PI / 180.0;
		Topology::Rotate(PI / 2 - angle, dx, dy, pt.x, pt.y);
		trajectory.push_back(pt);
		//pt.angle = //angle - m_lcmMsgRefTrajectory.angle[i];
		//double dis = pt.x*pt.x + pt.y*pt.y;
		//if (min > dis) {
		//	min = dis;
		//	minIndex = i;
		//}
	}
	/*car.x = curpt.x - m_lcmMsgRefTrajectory.x[0];
	car.y = curpt.y - m_lcmMsgRefTrajectory.y[0];*/
	RoadPoint pt;
	double dx = curpt.y - m_lcmMsgRefTrajectory.x[0];
	double dy = curpt.x - 500000 - m_lcmMsgRefTrajectory.y[0];
	// *PI / 180.0;
	Topology::Rotate(PI / 2 - angle, dx, dy, pt.x, pt.y);
	car.x = pt.x;
	car.y = pt.y;
	car.angle = PI / 2;
	return trajectory;
}
std::vector<RoadPoint> DataCenter::GetRefTrajectories()
{
	QuickLock lk(m_lockRefTrajectory);
	std::vector<RoadPoint> pos;
	for (int i = 0; i < m_lcmMsgRefTrajectory.num; i++) {
		RoadPoint p;
		p.x = m_lcmMsgRefTrajectory.y[i];
		p.y = m_lcmMsgRefTrajectory.x[i];
		p.angle = PI / 2 - m_lcmMsgRefTrajectory.angle[i]*PI/180;
		pos.push_back(p);
	}
	return pos;
}


std::vector<std::vector<RoadPoint>> DataCenter::GetMultiLanes(int & laneIndex){
	QuickLock lk(m_lockMultiLane);
	laneIndex = m_lcmMsgMultiLane.index;
	std::vector < std::vector<RoadPoint>> lanes;
	for (int i = 0; i < m_lcmMsgMultiLane.lanenum; i++)
	{
		std::vector<RoadPoint> pos;
		for (int j = 0; j < m_lcmMsgMultiLane.pointnum; j++)
		{
			RoadPoint p;
			p.x = m_lcmMsgMultiLane.y[i][j];
			p.y = m_lcmMsgMultiLane.x[i][j];
			p.angle = PI / 2 - m_lcmMsgMultiLane.yaw[i][j] * PI / 180;
			if (p.x == -1000)
			{
				continue;
			}
			pos.push_back(p);
		}
		lanes.push_back(pos);
	}
	return lanes;
}

std::vector<RoadPoint> DataCenter::GetCamLanes()
{
	QuickLock lk(m_lockCamLane);
	const double dx = -0.1;
	const double dy = +1.5;
	if (m_lcmMsgCamLane.leftnum < 5 && m_lcmMsgCamLane.rightnum < 5) {
		return std::vector<RoadPoint>();
	}
	if (m_lcmMsgCamLane.leftnum < 5) {
		std::vector<RoadPoint> rpts;
		double tdx = m_lcmMsgCamLane.rightx[0];
		for (int i = 0; i < m_lcmMsgCamLane.rightnum; i++) {
			RoadPoint pt;
			pt.x = m_lcmMsgCamLane.rightx[i] + dx - tdx;
			pt.y = m_lcmMsgCamLane.righty[i] + dy;
			rpts.push_back(pt);
		}
		SXYSpline spline;
		spline.init(rpts);
		rpts.clear();
		for (int i = 0; i < 20; i++) {
			RoadPoint rpt;
			spline.getXY(i, rpt.x, rpt.y);
			double deltaX, deltaY;
			spline.getDeriveXY(i, deltaX, deltaY);
			rpt.angle = atan2(deltaY, deltaX);
			rpts.push_back(rpt);
		}
		return rpts;
	}
	if (m_lcmMsgCamLane.rightnum < 5) {
		std::vector<RoadPoint> rpts;
		double tdx = m_lcmMsgCamLane.leftx[0];
		for (int i = 0; i < m_lcmMsgCamLane.leftnum; i++) {
			RoadPoint pt;
			pt.x = m_lcmMsgCamLane.leftx[i] + dx - tdx;
			pt.y = m_lcmMsgCamLane.lefty[i] + dy;
			rpts.push_back(pt);
		}
		SXYSpline spline;
		spline.init(rpts);
		rpts.clear();
		for (int i = 0; i < 20; i++) {
			RoadPoint rpt;
			spline.getXY(i, rpt.x, rpt.y);
			double deltaX, deltaY;
			spline.getDeriveXY(i, deltaX, deltaY);
			rpt.angle = atan2(deltaY, deltaX);
			rpts.push_back(rpt);
		}
		return rpts;
	}
	std::vector<RoadPoint> rptsR;
	{
		SXYSpline spline;
		double tdx = 0;// m_lcmMsgCamLane.rightx[0];
		RoadPoint strpt;
		strpt.x = m_lcmMsgCamLane.rightx[0];
		strpt.y = -dy;
		rptsR.push_back(strpt);
		for (int i = 0; i < m_lcmMsgCamLane.rightnum; i++) {
			RoadPoint pt;
			pt.x = m_lcmMsgCamLane.rightx[i] + dx - tdx;
			pt.y = m_lcmMsgCamLane.righty[i] + dy;
			rptsR.push_back(pt);
		}
		spline.init(rptsR);
		rptsR.clear();
		for (int i = 0; i < 20; i++) {
			RoadPoint rpt;
			spline.getXY(i, rpt.x, rpt.y);
			double deltaX, deltaY;
			spline.getDeriveXY(i, deltaX, deltaY);
			rpt.angle = atan2(deltaY, deltaX);
			rptsR.push_back(rpt);
		}
	}
	std::vector<RoadPoint> rptsL;
	{
		SXYSpline spline;
		double tdx = 0;// m_lcmMsgCamLane.leftx[0];
		RoadPoint strpt;
		strpt.x = m_lcmMsgCamLane.leftx[0] + dx;
		strpt.y = 0;
		rptsL.push_back(strpt);
		for (int i = 0; i < m_lcmMsgCamLane.leftnum; i++) {
			RoadPoint pt;
			pt.x = m_lcmMsgCamLane.leftx[i] + dx - tdx;
			pt.y = m_lcmMsgCamLane.lefty[i] + dy;
			rptsL.push_back(pt);
		}
		spline.init(rptsL);
		rptsL.clear();
		for (int i = 0; i < 20; i++) {
			RoadPoint rpt;
			spline.getXY(i, rpt.x, rpt.y);
			double deltaX, deltaY;
			spline.getDeriveXY(i, deltaX, deltaY);
			rpt.angle = atan2(deltaY, deltaX);
			rptsL.push_back(rpt);
		}
	}
	std::vector<RoadPoint> rpts;
	//for (RoadPoint rpt : rptsL){
	//	rpts.push_back(rpt);
	//}
	//for (RoadPoint rpt : rptsR){
	//	rpts.push_back(rpt);
	//}
	//return rptsR;
	for (int i = 0; i < 20; i++) {
		RoadPoint rpt;
		rpt.x = (rptsL[i].x + rptsR[i].x)/2;
		rpt.y = (rptsL[i].y + rptsR[i].y) / 2;
		rpts.push_back(rpt);
	}
	return rpts;
}

PosPoint DataCenter::GetStopLine(double& width){
	QuickLock lk(m_lockMultiLane);

	//if (m_lcmMsgMultiLane.stopline){
	//	PosPoint p;
	//	p.x = m_lcmMsgMultiLane.sly;
	//	p.y = m_lcmMsgMultiLane.slx;
	//	p.angle = PI / 2 - m_lcmMsgMultiLane.slyaw*PI/180;
	//	width = m_lcmMsgMultiLane.slwidth;
	//	return p;
	//}
	//else{
	//	width = 0;
	//	return PosPoint();
	//}
	return PosPoint();
}


std::vector<RoadPoint> DataCenter::GetRefTrajectory()
{
	QuickLock lk(m_lockRefTrajectory);
	std::vector<RoadPoint> trajectory;
	trajectory.reserve(m_lcmMsgRefTrajectory.num);
	PosPoint curpt = GetCurPosition();
	double angle = curpt.angle;
	double min = DBL_MAX;
	int minIndex = 0;
	for (int i = 0; i < m_lcmMsgRefTrajectory.num; i++)
	{
		RoadPoint pt;
		double dx = m_lcmMsgRefTrajectory.x[i] - curpt.y;// m_lcmMsgRefTrajectory.x[0];
		double dy = m_lcmMsgRefTrajectory.y[i] - curpt.x;// m_lcmMsgRefTrajectory.y[0];
		// *PI / 180.0;
		Topology::Rotate(PI / 2 - angle, dx, dy, pt.x, pt.y);
		pt.angle = angle - m_lcmMsgRefTrajectory.angle[i];
		trajectory.push_back(pt);
		double dis = pt.x*pt.x + pt.y*pt.y;
		if (min > dis) {
			min = dis;
			minIndex = i;
		}
	}
	
	trajectory.erase(trajectory.begin(), trajectory.begin() + minIndex);
	PosPoint traPt;
	traPt.x = trajectory[1].x - trajectory[0].x;
	traPt.y = trajectory[1].y - trajectory[0].y;
	double length = sqrt(Topology::Distance2(trajectory[0], trajectory[1]));
	PosPoint carPt;
	carPt.x = -trajectory[0].x;// -curPt.x;
	carPt.y = -trajectory[0].y;// -curPt.y;
	double bottom = (traPt.x*carPt.x + traPt.y*carPt.y) / length;
	double dx = traPt.x*bottom / length;
	double dy = traPt.y*bottom / length;
	trajectory[0].x += dx;
	trajectory[0].y += dy;


	//double min = DBL_MAX;
	//double minIndex = 0;
	//PosPoint vec;
	//for (int i = 0; i <= 9; i++)
	//{

	//	double tmp1 =  trajectory[i].x*trajectory[i].x+trajectory[i].y*trajectory[i].y; 
	//	PosPoint pt;
	//	pt.x = trajectory[i + 1].x - trajectory[i].x;
	//	pt.y = trajectory[i + 1].y - trajectory[i].y;
	//	double tmp = sqrt(pt.x*pt.x + pt.y*pt.y);
	//	double res = (trajectory[i].x*pt.x + pt.y * trajectory[i].y);
	//	double c = (res) / tmp;
	//	
	//	double dis = sqrt(tmp1*tmp1 - c*c);
	//	if (min > dis){
	//		min = dis;
	//		minIndex = i;
	//		vec.x = trajectory[i].x + c*pt.x / tmp;
	//		vec.y = trajectory[i].y + c*pt.y / tmp;

	//	}
	//}

	//trajectory.erase(trajectory.begin(), trajectory.begin() + minIndex);
	///*std::cout <<std::setprecision(10)<< "z:." << m_lcmMsgRefTrajectory.x[1] << "/" <<
	//	m_lcmMsgRefTrajectory.y[1] << "/" << m_lcmMsgRefTrajectory.x[2] << "/" << m_lcmMsgRefTrajectory.y[2] << std::endl;
	//*///if (i == 1)
	//trajectory[0].x = vec.x;
	//trajectory[0].y = vec.y;
	//{
	//	trajectory[0].angle = atan((trajectory[1].y - trajectory[0].y) / (trajectory[1].x - trajectory[0].x));
	//}
	return trajectory;
}

void DataCenter::Get_InitAngle_Qi(SXYSpline* spline, double& angle, double& qi)
{
	double dx0, dy0;
	spline->getDeriveXY(0, dx0, dy0);
	double first_angle = Topology::toAngle(dx0, dy0);
	angle = PI / 2 - first_angle;
	////QuickLock lk(m_lockRefTrajectory);
	////std::vector<RoadPoint> trajectory = GetRefTrajectory();
	////QuickLock lk(m_lockRefTrajectory);
	////double first_pt_angle = trajectory[0].angle;
	////PosPoint curpos = GetCurPosition();
	//RadAngle cur_car_angle = PI / 2;// GetCurPosition().angle;
	////double car_x = curpos.y;
	//// car_y = curpos.x-500000;
	////if (first_pt_angle < 0)
	////{
	////	first_pt_angle += PI;
	////angle = (RadAngle)(cur_car_angle - first_pt_angle);
	////}
	////这里不确定是不是采用绝对值，如果不对再改一下
	///*angle =(RadAngle)( cur_car_angle - first_pt_angle);*///current car angle respect to first point on reference trajectory
	////std::cout << "cur" << cur_car_angle << "\tz:" << first_pt_angle << std::endl;
	////这里只从参考轨迹前10个点搜索最近点
	///*double min = DBL_MAX;
	//double minIndex = 0;
	//for (int i = 0; i <= 10; i++)
	//{
	//double tmp = (car_x - m_lcmMsgRefTrajectory.x[i]) * (car_x - m_lcmMsgRefTrajectory.x[i]) +
	//(car_y - m_lcmMsgRefTrajectory.y[i]) * (car_y - m_lcmMsgRefTrajectory.y[i]);
	//if (min > tmp){
	//min = tmp;
	//minIndex = i;
	//}
	//}*/
	qi = 0;
	return;
	double min = spline->X[0] * spline->X[0] +spline->Y[0]*spline->Y[0];
	if (spline->X[0] < 0){
		qi = -sqrt(min);
	}
	else{
		qi = sqrt(min);
	}
	//angle = (RadAngle)(cur_car_angle - ((trajectory[0].angle < 0 ? PI : 0) + trajectory[0].angle));

}

std::vector<RoadPoint> DataCenter::GetRefTrajectory_Qi(double& qi)
{
	std::vector<RoadPoint> refTrajectory = GetRefTrajectories();
	std::vector<RoadPoint> trajectory;
	trajectory.reserve(refTrajectory.size());
	PosPoint curpt = GetCurPosition();
	//double min = DBL_MAX;
	//int minIndex = 0;
	//转到车体坐标系
	for (int i = 0; i < refTrajectory.size(); i++)
	{
		PosPoint pt;
		CoordTransform::WorldToLocal(curpt, refTrajectory[i], &pt);
		trajectory.push_back(pt);
		/*double dis = pt.x*pt.x + pt.y*pt.y;
		if (min > dis&&cos(pt.angle - curpt.angle) > 0) {
			min = dis;
			minIndex = i;
		}*/
	}

	qi=CoordTransform::TrimLocalPathToCurPt(trajectory);
	////求参考轨迹上离车当前点最近的点
	//std::pair<double, double> v1;
	//v1.first = trajectory[minIndex + 1].x - trajectory[minIndex].x;
	//v1.second = trajectory[minIndex + 1].y - trajectory[minIndex].y;
	//RoadPoint firstPt;
	//firstPt.y = (-v1.first * v1.second * trajectory[minIndex].x 
	//	+ v1.first * v1.first * trajectory[minIndex].y) 
	//	/ (v1.first * v1.first + v1.second * v1.second);
	//firstPt.x = (-v1.second * firstPt.y) / (v1.first);
	////firstPt.angle = trajectory[minIndex].angle;
	//firstPt.angle = atan2(v1.second, v1.first);
	//
	////std::cout << "angle: " << tan(firstPt.angle) << std::endl;
	//trajectory.erase(trajectory.begin(), trajectory.begin() + minIndex);
	//trajectory[0] = firstPt;
	//qi = sqrt(trajectory[0].x * trajectory[0].x + trajectory[0].y * trajectory[0].y);
	//if (firstPt.x < 0)
	//{
	//	qi = -qi;
	//}
	return trajectory;
}

void DataCenter::GetLanes_Qi(int laneIndex,double& qi, std::vector<RoadPoint>& lane){
	
}

bool DataCenter::WaitForLocation(unsigned int milliseconds){
	QuickLock lk(m_lockLocation);
	return m_waitLocation.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForStatusBody(unsigned int milliseconds){
	QuickLock lk(m_lockStatusBody);
	return m_waitStatusBody.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForVeloGrid(unsigned int milliseconds){
	QuickLock lk(m_lockVeloGrid);
	return m_waitVeloGrid.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForCurb(unsigned int milliseconds){
	QuickLock lk(m_lockCurb);
	return m_waitCurb.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForRefTrajectory(unsigned int milliseconds)
{
	QuickLock lk(m_lockRefTrajectory);

	return m_waitRefTrajectory.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForMultiLane(unsigned int milliseconds){

	QuickLock lk(m_lockMultiLane);

	return m_waitMultiLane.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForCamLane(unsigned int milliseconds)
{
	QuickLock lk(m_lockCamLane);
	return m_waitCamLane.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForScene(unsigned int milliseconds)
{
	QuickLock lk(m_lockScene);
	return m_waitScene.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::HasLocation(){
	return m_lcmLocation.HasLcmMessage();
}

bool DataCenter::HasStatusBody(){
	return m_lcmStatusBody.HasLcmMessage();
}

bool DataCenter::HasVeloGrid(){
	return m_lcmVeloGrid.HasLcmMessage();
}

bool DataCenter::HasCurb(){
	return m_lcmCurb.HasLcmMessage();
}

bool DataCenter::HasRefTrajectory()
{
	return m_lcmRefTrajectory.HasLcmMessage();
}

bool DataCenter::HasMultiLane()
{
	return m_lcmMultiLane.HasLcmMessage();
}

bool DataCenter::HasCamLane()
{
	return m_lcmCamLane.HasLcmMessage();
}

bool DataCenter::HasSceneMsg()
{
	return m_lcmScene.HasLcmMessage();
}

