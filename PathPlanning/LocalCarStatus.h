#pragma once
#ifndef _PATH_PLANNING_LOCAL_CAR_STATUS_H
#define _PATH_PLANNING_LOCAL_CAR_STATUS_H

#include "BaseCarStatus.h"
class LocalPosThread;
class LocalCarStatus :
	public BaseCarStatus
{
private:
	friend class LocalPosThread;
	LocalPosThread * thread;
protected:
	LocalCarStatus();
	~LocalCarStatus();
public:
	static LocalCarStatus& GetInstance();
	void Start();
	void Suspend();
	void Resume();
	void End();
};

#endif // !_PATH_PLANNING_LOCAL_CAR_STATUS_H
