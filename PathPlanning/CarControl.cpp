#include "CarControl.h"
#include "lcmtype\LcmSet.h"
#include "DataCenter.h"


CarControl::CarControl()
{
	m_lcmControl.intialSend(LCM_NET_CONTROL_COMMAND, LCM_CHANNEL_CONTROL_COMMAND);
}

CarControl::~CarControl()
{
	m_lcmControl.uninitialLcm();
}

CarControl & CarControl::GetInstance()
{
	static CarControl instance;
	return instance;
}

void CarControl::SendCommand(CarInfo info)
{
	ControlCommand_t msg;
	msg.gear = info.gear;
	msg.vehicleSpeed = info.speed;
	msg.wheelAngle = info.steerAngle;
	msg.controlStatus = info.state;
	m_lcmControl.sendLcm(&msg);
}

void CarControl::StopCommand()
{
	ControlCommand_t msg;
	CarInfo info = DataCenter::GetInstance().GetCarInfo();
	msg.gear = info.gear;
	msg.vehicleSpeed = info.speed;
	msg.wheelAngle = info.steerAngle;
	msg.controlStatus = ERunState::E_STOP;
	m_lcmControl.sendLcm(&msg);
}
