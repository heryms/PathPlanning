#include "testModule.h"
#include <iostream>
#include <iomanip>
using namespace std;
void test_To_Guassian(double grad, double minute, double second,
	double grad1, double minute1, double second1){
	double lon = grad + minute / 60.0 + second / 3600.0;
	double lat = grad1 + minute1 / 60.0 + second1 / 3600;
	cout << "lon:" << lon << "lat:" << lat << endl;
	double x = 0, y = 0;
	LBtoxy test(lon, lat, 6);
	test.calculateAll();
	x = test.getx();
	y = test.gety() + 500000;
	//CoordSystem coord_system = WGS84;
	//CoordTransform::LongLat2XY(lon, lat, coord_system, x, y);
	//double real_y = gt_x, real_x = gt_y;
	cout << setiosflags(ios::fixed)<<"x:"<<setprecision(10) << x << "y:" << y << endl;
	//cout << setiosflags(ios::fixed) << "delta_x:" << setprecision(10) << (real_x - x) 
		//<< "delta_y:" << real_y - y << endl;
	//double reverse_lon = 0, reverse_lat = 0;
	//CoordTransform::XY2LongLat(x, y, coord_system, reverse_lon, reverse_lat);
	//cout << setiosflags(ios::fixed) << "x:" << setprecision(10) << reverse_lon << "y:" << reverse_lat - lat<< endl;
}
void test_To_Guassian(
	double lon, double lat
	){
	LBtoxy test(lon,lat,6);
	test.calculateAll();
	cout << setiosflags(ios::fixed) << setprecision(10)<< test.getx() << "/" << test.gety() + 500000 << endl;
}


void test_Clothoid()
{
	RoadPoint *rdPt = new RoadPoint[100];

	Clothoid clo(0.0, 0.0, 0.0, 50.0, 50.0, PI / 2.0);
	clo.PointsOnClothoid(rdPt, 100);
}