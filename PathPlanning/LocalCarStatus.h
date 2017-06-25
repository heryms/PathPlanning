#pragma once
#ifndef _PATH_PLANNING_LOCAL_CAR_STATUS_H
#define _PATH_PLANNING_LOCAL_CAR_STATUS_H

#include "BaseCarStatus.h"

class LocalCarStatus :
	public BaseCarStatus
{
private:
	friend class LocalPosThread;
	LocalPosThread * thread;
	PosPoint orgPos;
protected:
	LocalCarStatus();
	~LocalCarStatus();
public:
	static LocalCarStatus& GetInstance();
	void SetOriginPos(PosPoint pos);
	void Start();
	void Suspend();
	void Resume();
	void End();
	PosPoint GetPosition();	
	double GetSpeed();
	double GetSteerAngle();
};

#endif // !_PATH_PLANNING_LOCAL_CAR_STATUS_H
