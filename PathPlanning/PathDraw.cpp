#include "PathDraw.h"
#include "lcmtype\LcmSet.h"


PathDraw::PathDraw()
{
	m_lcmDraw.intialSend(LCM_NET_DECISION_DRAW, LCM_CHANNEL_DECISION_DRAW);
	m_lcmPaths.intialSend(LCM_NET_CANDIDATES, LCM_CHANNEL_CANDIDATES);
	m_lcmSelected.intialSend(LCM_NET_SELECTEDPATH, LCM_CHANNEL_SELECTEDPATH);
}


PathDraw::~PathDraw()
{
}

void PathDraw::SendDraw(DecisionDraw_t& info)
{
	m_lcmDraw.sendLcm(&info);
}

void PathDraw::SendDraw(std::vector<std::vector<RoadPoint>>& paths, std::vector<RoadPoint> & selected)
{
	Draw_t draw;

	for (auto& path : paths)
	{
		for (RoadPoint& rpt : path)
		{
			draw.x.push_back(rpt.x);
			draw.y.push_back(rpt.y);
		}
	}
	draw.ptnum = draw.x.size();
	m_lcmPaths.sendLcm(&draw);

	draw.x.clear();
	draw.y.clear();
	for (RoadPoint& rpt : selected){
		draw.x.push_back(rpt.x);
		draw.y.push_back(rpt.y);
	}
	draw.ptnum = draw.x.size();
	m_lcmSelected.sendLcm(&draw);


}

void PathDraw::DrawCandidates(std::vector<std::vector<RoadPoint>>& paths)
{
	Draw_t draw;

	for (auto& path : paths)
	{
		for (RoadPoint& rpt : path)
		{
			draw.x.push_back(rpt.x);
			draw.y.push_back(rpt.y);
		}
	}
	draw.ptnum = draw.x.size();
	m_lcmPaths.sendLcm(&draw);
}
void PathDraw::DrawSelected(std::vector<RoadPoint> & selected)
{
	Draw_t draw;

	for (RoadPoint& rpt : selected){
		draw.x.push_back(rpt.x);
		draw.y.push_back(rpt.y);
	}
	draw.ptnum = draw.x.size();
	m_lcmSelected.sendLcm(&draw);
}