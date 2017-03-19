#pragma once
/************************************************************************/
/*  class for coordinate transform
    created:2017/3/19 
	created by heryms zjm
	remain tested
	untested
*/
/************************************************************************/

// define a camera parameter struct 

typedef struct {
	int width;
	int height;

	double cam_pos_x;
	double cam_pos_y;
	double cam_pos_z;
	double roll;
	double pitch;
	double yaw;

	double f;//f=fy
	double fx;
	double fy;
	double cx;
	double cy;

}CamParam, *ptrCamParam;
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

	// image to road
	static int ImageToRoad(CamParam *pCamParam, int iIimage, int iJimage, double *pdIRoad, double *pdJRoad);
	static double calcYcameraFromUimage(CamParam *pCamParam, double dUimage);
	static double calcZcameraFromYcameraOnRoad(CamParam *pCamParam, double dYcamera);
	static double calcZvehicleFromUimage(CamParam *pCamParam, double dUimage);
	static double calcXvehicleFromVimageAndZvehicle(CamParam *pCamParam, double dVimage, double dZvehicle);
	static double calcXcameraFromVimage(CamParam *pCamParam, double dVimage);
	static int calciIsrcFromZcameraOnRoad(CamParam *pCamParam, double dZcamera);
	static double calcVimageFromXcameraAndZcameraOnRoad(CamParam *pCamParam, double dXcameraRoad, double dZcameraRoad);
	static int calciJsrcFromXvehicleAndZvehicleOnRoad(CamParam *pCamParam, double dXvehicleOnRoad, double dZvehicleOnRoad);
};

