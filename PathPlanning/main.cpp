/*
	author: heryms
	created on 2017-3-18
*/
#include<iostream>
#include "CoordTransform.h"
#include "BaseType.h"
#include "Variables.h"
#include <fstream>
#include "testModule.h"
#include <vector>
#include "DataCenter.h"
#include "PathGenerate.h"
#include <chrono>
int main()
 {
	test_axis_transform();
	//test_spline();
	//CoordTransform transform;
	//test_Clothoid();
	DataCenter::GetInstance().StartVeloGrid();
	//DataCenter::GetInstance().StartCurb();
	DataCenter::GetInstance().StartStatusBody();
	DataCenter::GetInstance().StartRefTrajectory();
	DataCenter::GetInstance().StartLocation();
	DataCenter::GetInstance().StartMultiLane();
	__thread_sleep_for(1000);
	PathGenerate pathGen;
	while (true)
	{
		if (!DataCenter::GetInstance().WaitForVeloGrid(20)) {
			continue;
		}
		std::chrono::steady_clock::time_point startTime
			= std::chrono::steady_clock::now();
		pathGen.short_time_uturn();
		std::chrono::steady_clock::time_point endTime
			= std::chrono::steady_clock::now();
		std::chrono::milliseconds time
			= std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		std::cout << time.count() << std::endl;
		__thread_sleep_for(10);
	}
	DataCenter::GetInstance().EndAllSensor();
	return 0;
}