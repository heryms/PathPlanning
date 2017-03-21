#pragma once
#ifndef _PATH_PLANNING_PATH_DRAW_H
#define _PATH_PLANNING_PATH_DRAW_H

#include "lcmtype\DecisionDraw_t.hpp"
#include "lcmtype\LcmType_Handler.h"

using ckLcmType::DecisionDraw_t;
class PathDraw
{
public:
	PathDraw();
	~PathDraw();
	LcmHandler<DecisionDraw_t> m_lcmDraw;
	void SendDraw(DecisionDraw_t& info);
};

#endif // !_PATH_PLANNING_MAP_DRAW_H
