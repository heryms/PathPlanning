#pragma once
#ifndef _PATH_PLANNING_PATH_DRAW_H
#define _PATH_PLANNING_PATH_DRAW_H

#include "lcmtype\DecisionDraw_t.hpp"
#include "lcmtype\LcmType_Handler.h"
#include "lcmtype\Draw_t.hpp"
#include <vector>
#include "BaseType.h"

using ckLcmType::DecisionDraw_t;
using ckLcmType::Draw_t;

class PathDraw
{
public:
	PathDraw();
	~PathDraw();
	LcmHandler<DecisionDraw_t> m_lcmDraw;
	LcmHandler<Draw_t> m_lcmPaths;
	LcmHandler<Draw_t> m_lcmSelected;
	void SendDraw(std::vector<std::vector<RoadPoint>>& paths, std::vector<RoadPoint> & selected);
	void SendDraw(DecisionDraw_t& info);
	void DrawCandidates(std::vector<std::vector<RoadPoint>>& paths);
	void DrawSelected(std::vector<RoadPoint> & selected);
};

#endif // !_PATH_PLANNING_MAP_DRAW_H
