#pragma once
/************************************************************************/
/*  class for coordinate transform
    created:2017/3/19 
	untested
*/
/************************************************************************/


typedef struct
{
	double  X;
	double  Y;
	double  Angle;
}OriginPt;

enum CoordSystem
{
	WGS84 = 0,
	Beijing54,
	Xian80
};
class CoordTransform
{
public:
	static int LongLat2XY(double lon, double lat, CoordSystem coordSys, double &X, double &Y);
	static int XY2LongLat(double X, double Y, CoordSystem coordSys, double& lon, double& lat);
	static int WorldtoMap(OriginPt org, double xIn, double yIn, double &xOut, double &yOut);
	static int MaptoWorld(OriginPt org, double xIn, double yIn, double &xOut, double &yOut);
};

