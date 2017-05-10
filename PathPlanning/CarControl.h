#pragma once
#ifndef _PATH_PLANNING_CAR_CONTROL_H
#define _PATH_PLANNING_CAR_CONTROL_H

#include "lcmtype\controlcommand_t.hpp"
#include "lcmtype\LcmType_Handler.h"
#include "BaseType.h"
using ckLcmType::ControlCommand_t;
class CarControl
{
protected:
	CarControl();
	~CarControl();
	LcmHandler<ControlCommand_t> m_lcmControl;
public:
	static CarControl& GetInstance();
	void SendCommand(CarInfo info);
	void StopCommand();
};


#endif // !_PATH_PLANNING_CAR_CONTROL_H