#pragma once
#include "BaseType.h"
/************************************************************************/
/*  class for coordinate transform
    created:2017/3/19 
	created by heryms zjm
	remain tested
	untested
*/
/************************************************************************/

// define a camera parameter struct 


enum CoordSystem
{
	WGS84 = 0,
	Beijing54,
	Xian80
};
class CoordTransform
{
public:
	static int WorldtoLocal(PosPoint org, double xIn, double yIn, double &xOut, double &yOut);
	static int LocaltoWorld(PosPoint org, double xIn, double yIn, double &xOut, double &yOut);

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

