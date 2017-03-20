#ifndef _PATH_PLANNING_DATA_CENTER_H
#define _PATH_PLANNING_DATA_CENTER_H
#include "BaseType.h"
#include "lcmtype\Location_t.hpp"
#include "lcmtype\StatusBody_t.hpp"
#include "lcmtype\LcmRecvHelper.h"
#include "lcmtype\lasercurb_t.hpp"
#include "lcmtype\VeloGrid_t.hpp"

using ckLcmType::StatusBody_t;
using ckLcmType::Location_t;
using ckLcmType::VeloGrid_t;
using ckLcmType::cloudHandler;

class DataCenter
{
private:
	PosPoint m_curPos;//x,y in Gauss; orientation by x
	CarInfo m_curCarInfo;//speed in km/h; steerAngle in deg
	LcmRecvHelper<Location_t> m_lcmLocation;
	LcmRecvHelper<StatusBody_t> m_lcmStatusBody;
	LcmRecvHelper<VeloGrid_t> m_lcmVeloGrid;
	LcmRecvHelper<cloudHandler> m_lcmCurb;
	Location_t m_lcmMsgLocation;
	StatusBody_t m_lcmMsgStatusBody;
	VeloGrid_t m_lcmMsgVeloGrid;
	cloudHandler m_lcmMsgCurb;
	/*triggered while receiving Location_t */
	void LocationRecvOperation(const Location_t* msg, void*);
	/*triggered while receiving StatusBody */
	void StatusBodyRecvOperation(const StatusBody_t* msg, void*);
	/*triggered while receiving VeloGrid */
	void VeloGridRecvOperation(const VeloGrid_t* msg, void*);
	/*triggered while receiving Cloud */
	void CurbRecvOperation(const cloudHandler* msg, void*);
public:
	DataCenter();
	~DataCenter();
	/*start all sensors*/
	void StartAllSensor();
	/*end all sensors*/
	void EndAllSensor();
	/*@return x,y in Gauss, orientation by x*/
	PosPoint GetCurPosition();
	/*@return speed in km/h£¬ steerAngle in deg*/
	CarInfo GetCarInfo();
	/*@return if having position during 500ms*/
	bool HasPosition();
	/*@return if having car info during 500ms*/
	bool HasCarInfo();
	/*@return if having velo grid during 500ms*/
	bool HasVeloGrid();
	/*@return if having curb during 500ms*/
	bool HasCurb();
private:

};

#endif // !_PATH_PLANNING_DATA_CENTER_H


