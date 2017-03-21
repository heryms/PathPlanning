#pragma once
#ifndef _PATH_PLANNING_CAR_CONTROL_H
#define _PATH_PLANNING_CAR_CONTROL_H

#include "lcmtype\controlcommand_t.hpp"
#include "lcmtype\LcmType_Handler.h"
#include "BaseType.h"
using ckLcmType::ControlCommand_t;
class CarControl
{
public:
	CarControl();
	~CarControl();
	LcmHandler<ControlCommand_t> m_lcmControl;
	void SendCommand(CarInfo info);
};


#endif // !_PATH_PLANNING_CAR_CONTROL_H