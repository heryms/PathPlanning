/*
	author: heryms
	created on 2017-3-18
*/
#include<iostream>
#include "CoordTransform.h"
#include "Clothoid.h"
#include "BaseType.h"
#include "Variables.h"
#include <fstream>

int main()
{
	//CoordTransform transform;
	
	RoadPoint *roadPt = new RoadPoint[100];

	Clothoid clo(0.0, 0.0, 0.0, 50.0, 50.0, PI / 2.0);
	clo.PointsOnClothoid(roadPt, 100);


	/*ofstream out("1.txt", std::ios::out);*/
	for (int i = 0; i < 100; i++)
	{
		std::cout << i << " " << roadPt[i].x << " " << roadPt[i].y << " " << roadPt[i].angle << std::endl;
	}


	return 0;
}