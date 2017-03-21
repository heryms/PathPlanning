#include "PathDraw.h"
#include "lcmtype\LcmSet.h"


PathDraw::PathDraw()
{
	m_lcmDraw.intialSend(LCM_NET_DECISION_DRAW, LCM_CHANNEL_DECISION_DRAW);
}


PathDraw::~PathDraw()
{
}

void PathDraw::SendDraw(DecisionDraw_t& info)
{
	m_lcmDraw.sendLcm(&info);
}
