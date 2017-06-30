#pragma once
#include "BaseType.h"
#include "Variables.h"
#include <vector>
/************************************************************************/
/*  class for coordinate transform
    created:2017/3/19 
	created by heryms zjm
	remain tested
	
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
	/**
	* 笛卡尔坐标系，方向角与X轴夹角。
	* org的local坐标为(0,0,PI/2)
	**/ 
	static void WorldToLocal(PosPoint org, PosPoint ptIn, PosPoint * ptOut);
	/**
	* 笛卡尔坐标系，方向角与X轴夹角。
	* org的local坐标为(0,0,PI/2)
	**/
	static void LocalToWorld(PosPoint org, PosPoint ptIn, PosPoint * ptOut);
	/**
	* 精确当前位置(0,0,PI/2)在路径中的最近点，并返回距离
	**/
	static double TrimLocalPathToCurPt(std::vector<RoadPoint>& localPath);
	//static int WorldtoLocal(PosPoint org, double xIn, double yIn, double &xOut, double &yOut);
	//static int LocaltoWorld(PosPoint org, double xIn, double yIn, double &xOut, double &yOut);
	static bool LocaltoGrid(PosPoint org, int &xOut, int &yOut);
	static bool GridtoLocal(int xIn, int yIn, double &xOut, double &yOut);
	static std::vector<int> XTriangleToGrids(PosPoint x1, PosPoint x2, PosPoint y, PosPoint yGrid);
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

