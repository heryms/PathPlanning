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
int main()
{
	//CoordTransform transform;
	//test_Clothoid();
	DataCenter::GetInstance().StartAllSensor();
	__thread_sleep_for(1000);
	PathGenerate pathGen;
	while (true)
	{
		if (!DataCenter::GetInstance().WaitForVeloGrid(20)) {
			return;
		}
		pathGen.path_generate();
	}
	DataCenter::GetInstance().EndAllSensor();
	return 0;
}