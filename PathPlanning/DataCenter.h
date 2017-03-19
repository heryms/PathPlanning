#ifndef _PATH_PLANNING_DATA_CENTER_H
#define _PATH_PLANNING_DATA_CENTER_H
#include "BaseType.h"
#include "lcmtype\Location_t.hpp"
#include "lcmtype\StatusBody_t.hpp"
#include "lcmtype\LcmRecvHelper.h"

using ckLcmType::StatusBody_t;
using ckLcmType::Location_t;

class DataCenter
{
private:
	PosPoint m_curPos;//x,y in Gauss; orientation by x
	CarInfo m_curCarInfo;
	LcmRecvHelper<Location_t> m_lcmLocation;
	LcmRecvHelper<StatusBody_t> m_lcmStatusBody;
	Location_t m_lcmMsgLocation;
	StatusBody_t m_lcmMsgStatusBody;
	void LocationRecvOperation(const Location_t* msg, void*);
	void StatusBodyRecvOperation(const StatusBody_t* msg, void*);
	
public:
	DataCenter();
	~DataCenter();
	/*@return x,y in Gauss, orientation by x*/
	PosPoint GetCurPosition();
	CarInfo GetCarInfo();
private:

};

#endif // !_PATH_PLANNING_DATA_CENTER_H


