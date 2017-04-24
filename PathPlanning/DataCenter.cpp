#include "DataCenter.h"
#include "lcmtype\LcmSet.h"
#include "LocalCarStatus.h"
#include <cmath>

typedef std::unique_lock<std::mutex> QuickLock;

void DataCenter::RefTrajectoryRecvOperation(const PlanOutput * msg, void *)
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
	m_waitLocation.notify_all();
}

void DataCenter::StatusBodyRecvOperation(const StatusBody_t* msg, void*){
	QuickLock lk(m_lockStatusBody);
	m_lcmMsgStatusBody = *msg;
	LocalCarStatus::GetInstance().SetSteerAngle(msg->wheelAngle);
	LocalCarStatus::GetInstance().SetSpeed(msg->vehicleSpeed);
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

void DataCenter::StartAllSensor(){
	StartLocation();
	StartStatusBody();
	StartVeloGrid();
	StartCurb();
	StartRefTrajectory();
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

PosPoint DataCenter::GetCurPosition(){
	QuickLock lk(m_lockCurb);
	m_curPos.x = m_lcmMsgLocation.gau_pos[1] + 500000;
	m_curPos.y = m_lcmMsgLocation.gau_pos[0];
	m_curPos.angle = PI / 2 - m_lcmMsgLocation.orientation[2];
	return m_curPos;
}

CarInfo DataCenter::GetCarInfo(){
	QuickLock lk(m_lockStatusBody);
	m_curCarInfo.speed = m_lcmMsgStatusBody.vehicleSpeed;
	m_curCarInfo.steerAngle = m_lcmMsgStatusBody.wheelAngle;
	return m_curCarInfo;
}

VeloGrid_t& DataCenter::GetLidarData()
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

laserCurbs::pointXYZI DataCenter::GetRoadEdgeCoefficient(CurbDirection dir)
{
	QuickLock lk(m_lockCurb);
	laserCurbs::pointXYZI coeff;
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
	return laserCurbs::pointXYZI();
}

PosPoint DataCenter::GetTargetPoint() {
	return PosPoint();
}

std::vector<PointPt> DataCenter::GetRefTrajectory()
{
	QuickLock lk(m_lockRefTrajectory);
	std::vector<PointPt> trajectory;
	trajectory.reserve(m_lcmMsgRefTrajectory.tn);

	for (int i = 0; i < m_lcmMsgRefTrajectory.tn; i++)
	{
		PointPt pt;
		pt.x = m_lcmMsgRefTrajectory.TrajectoryX[i];
		pt.y = m_lcmMsgRefTrajectory.TrajectoryY[i];
		trajectory.push_back(pt);
	}
	return trajectory;
}

void DataCenter::Get_InitAngle_Qi(double& angle, double& qi)
{
	QuickLock lk0(m_lockLocation);
	QuickLock lk1(m_lockRefTrajectory);

	double first_pt_angle = m_lcmMsgRefTrajectory.TrajectoryAngle[0];
	double cur_car_angle = m_lcmMsgLocation.orientation[2];
	double car_x = m_lcmMsgLocation.gau_pos[0];
	double car_y = m_lcmMsgLocation.gau_pos[1];

	//这里不确定是不是采用绝对值，如果不对再改一下
	angle = abs(cur_car_angle - first_pt_angle);//current car angle respect to first point on reference trajectory

	//这里只从参考轨迹前10个点搜索最近点
	double min = 0;
	for (int i = 0; i < 10; i++)
	{
		double tmp = (car_x - m_lcmMsgRefTrajectory.TrajectoryX[i]) * (car_x - m_lcmMsgRefTrajectory.TrajectoryX[i]) +
			(car_y - m_lcmMsgRefTrajectory.TrajectoryY[i]) * (car_y - m_lcmMsgRefTrajectory.TrajectoryY[i]);
		if (min > tmp)
			min = tmp;
	}
	qi = sqrt(min);
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
	return m_lcmRefTrajectory.HasLcmMessage;
}

