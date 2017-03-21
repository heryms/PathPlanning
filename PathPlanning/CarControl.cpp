#include "CarControl.h"
#include "lcmtype\LcmSet.h"

CarControl::CarControl()
{
	m_lcmControl.intialSend(LCM_NET_CONTROL_COMMAND, LCM_CHANNEL_CONTROL_COMMAND);
}

CarControl::~CarControl()
{

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
