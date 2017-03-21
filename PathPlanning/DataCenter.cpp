#include "DataCenter.h"

#define LCM_NET_LOCATION ""
#define LCM_CHANNEL_LOCATION "CKLOCATION"
#define LCM_NET_STATUS_BODY ""
#define LCM_CHANNEL_STATUS_BODY "CKSTATUS"
#define LCM_NET_VELO_GRID ""
#define LCM_CHANNEL_VELO_GRID "CKVELOGRID"
#define LCM_NET_CLOUD ""
#define LCM_CHANNEL_CLOUD "CKCLOUD"

typedef std::unique_lock<std::mutex> QuickLock;

DataCenter::DataCenter()
{
}

DataCenter::~DataCenter()
{
}

void DataCenter::LocationRecvOperation(const Location_t* msg, void*){
	QuickLock lk(m_lockLocation);
	m_curPos.X = msg->gau_pos[1] + 500000;
	m_curPos.Y = msg->gau_pos[0];
	m_curPos.Angle = PI / 2 - msg->orientation[2];
	m_lcmMsgLocation = *msg;
	m_waitLocation.notify_all();
}

void DataCenter::StatusBodyRecvOperation(const StatusBody_t* msg, void*){
	QuickLock lk(m_lockStatusBody);
	m_curCarInfo.speed = msg->vehicleSpeed;
	m_curCarInfo.steerAngle = msg->wheelAngle;
	m_lcmMsgStatusBody = *msg;
	m_waitStatusBody.notify_all();
}

void DataCenter::VeloGridRecvOperation(const VeloGrid_t* msg, void*){
	QuickLock lk(m_lockVeloGrid);
	m_lcmMsgVeloGrid = *msg;
	m_waitVeloGrid.notify_all();
}

void DataCenter::CurbRecvOperation(const cloudHandler* msg, void*){
	QuickLock lk(m_lockCurb);
	m_waitCurb.notify_all();
	m_lcmMsgCurb = *msg;
}

void DataCenter::StartAllSensor(){
	m_lcmLocation.initialLcm(LCM_NET_LOCATION, LCM_CHANNEL_LOCATION
		, std::bind(&DataCenter::LocationRecvOperation, this,std::placeholders::_1,std::placeholders::_2),this);
	m_lcmStatusBody.initialLcm(LCM_NET_STATUS_BODY, LCM_CHANNEL_STATUS_BODY
		, std::bind(&DataCenter::StatusBodyRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
	m_lcmVeloGrid.initialLcm(LCM_NET_VELO_GRID, LCM_CHANNEL_VELO_GRID
		, std::bind(&DataCenter::VeloGridRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
	m_lcmCurb.initialLcm(LCM_NET_CLOUD, LCM_CHANNEL_CLOUD
		, std::bind(&DataCenter::CurbRecvOperation, this, std::placeholders::_1, std::placeholders::_2), this);
}

void DataCenter::EndAllSensor(){
	m_lcmLocation.uninitialLcm();
	m_lcmStatusBody.uninitialLcm();
	m_lcmVeloGrid.uninitialLcm();
	m_lcmCurb.uninitialLcm();
}

PosPoint DataCenter::GetCurPosition(){
	QuickLock lk(m_lockCurb);
	return m_curPos;
}

CarInfo DataCenter::GetCarInfo(){
	QuickLock lk(m_lockStatusBody);
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
	pt.Y = y;
	double x;
	switch (dir)
	{
	case LEFT:
		if (m_lcmMsgCurb.cloud[0].x)
		{
			x = -(m_lcmMsgCurb.cloud[0].y * y + m_lcmMsgCurb.cloud[0].z) / m_lcmMsgCurb.cloud[0].x;
			pt.X = x;
		}
		else
		{
			x = 0;
			pt.Y = -m_lcmMsgCurb.cloud[0].z / m_lcmMsgCurb.cloud[0].y;
			pt.X = x;
		}

		pt.Angle = atan(y / x);
		break;
	case RIGHT:
		if (m_lcmMsgCurb.cloud[1].x)
		{
			x = -(m_lcmMsgCurb.cloud[1].y * y + m_lcmMsgCurb.cloud[1].z) / m_lcmMsgCurb.cloud[1].x;
			pt.X = x;
		}
		else
		{
			x = 0;
			pt.Y = -m_lcmMsgCurb.cloud[1].z / m_lcmMsgCurb.cloud[1].y;
		}

		break;
	default:
		break;
	}

	double angle = atan(y / x);
	pt.Angle = RadAngle::Normalize(angle);
	return pt;
}

bool DataCenter::WaitForLocation(unsigned int milliseconds){
	QuickLock lk(m_waitLockLocation);
	return m_waitLocation.wait_for(lk,std::chrono::milliseconds(milliseconds))==std::cv_status::no_timeout;
}

bool DataCenter::WaitForStatusBody(unsigned int milliseconds){
	QuickLock lk(m_waitLockStatusBody);
	return m_waitStatusBody.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForVeloGrid(unsigned int milliseconds){
	QuickLock lk(m_waitLockVeloGrid);
	return m_waitVeloGrid.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
}

bool DataCenter::WaitForCurb(unsigned int milliseconds){
	QuickLock lk(m_waitLockCurb);
	return m_waitCurb.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout;
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

