#include "DataCenter.h"

#define LCM_NET_LOCATION ""
#define LCM_CHANNEL_LOCATION "CKLOCATION"
#define LCM_NET_STATUS_BODY ""
#define LCM_CHANNEL_STATUS_BODY "CKSTATUS"
#define LCM_NET_VELO_GRID ""
#define LCM_CHANNEL_VELO_GRID "CKVELOGRID"
#define LCM_NET_CLOUD ""
#define LCM_CHANNEL_CLOUD "CKCLOUD"

DataCenter::DataCenter()
{
}

DataCenter::~DataCenter()
{
}

void DataCenter::LocationRecvOperation(const Location_t* msg, void*){
	m_curPos.X = msg->gau_pos[1] + 500000;
	m_curPos.Y = msg->gau_pos[0];
	m_curPos.Angle = PI / 2 - msg->orientation[2];
	m_lcmMsgLocation = *msg;
}

void DataCenter::StatusBodyRecvOperation(const StatusBody_t* msg, void*){
	m_curCarInfo.speed = msg->vehicleSpeed;
	m_curCarInfo.steerAngle = msg->wheelAngle;
	m_lcmMsgStatusBody = *msg;
}

void DataCenter::VeloGridRecvOperation(const VeloGrid_t* msg, void*){
	m_lcmMsgVeloGrid = *msg;
}

void DataCenter::CurbRecvOperation(const cloudHandler* msg, void*){

}

void DataCenter::StartAllSensor(){
	m_lcmLocation.initialLcm(LCM_NET_LOCATION, LCM_CHANNEL_LOCATION, std::bind(&LocationRecvOperation, this,std::placeholders::_1),this);
	m_lcmStatusBody.initialLcm(LCM_NET_STATUS_BODY, LCM_CHANNEL_STATUS_BODY, std::bind(&StatusBodyRecvOperation, this, std::placeholders::_1), this);
	m_lcmVeloGrid.initialLcm(LCM_NET_VELO_GRID, LCM_CHANNEL_VELO_GRID, std::bind(&VeloGridRecvOperation, this, std::placeholders::_1), this);
	m_lcmCurb.initialLcm(LCM_NET_CLOUD, LCM_CHANNEL_CLOUD, std::bind(&VeloGridRecvOperation, this, std::placeholders::_1), this);
}

void DataCenter::EndAllSensor(){
	m_lcmLocation.uninitialLcm();
	m_lcmStatusBody.uninitialLcm();
	m_lcmVeloGrid.uninitialLcm();
	m_lcmCurb.uninitialLcm();
}

PosPoint DataCenter::GetCurPosition(){
	return m_curPos;
}

CarInfo DataCenter::GetCarInfo(){
	return m_curCarInfo;
}

bool DataCenter::HasPosition(){
	return m_lcmLocation.HasLcmMessage();
}

bool DataCenter::HasCarInfo(){
	return m_lcmStatusBody.HasLcmMessage();
}

bool DataCenter::HasVeloGrid(){
	return m_lcmVeloGrid.HasLcmMessage();
}

bool DataCenter::HasCurb(){
	return m_lcmCurb.HasLcmMessage();
}

VeloGrid_t DataCenter::GetLidarData()
{
	return m_lcmMsgVeloGrid;
}
