#include "CoordTransform.h"
#include <cmath>


#pragma region IMAGE_TO_ROAD_AND_ROAD_TO_IMAGE

double CoordTransform::calcZcameraFromYcameraOnRoad(CamParam *pCamParam, double dYcamera)
{
	double dZcamera = -pCamParam->fy * (pCamParam->cam_pos_y) / dYcamera;	
	return dZcamera;
}
double CoordTransform::calcZvehicleFromUimage(CamParam *pCamParam, double dUimage)	// Uimage->Zvehicle
{
	double dYcamera = calcYcameraFromUimage(pCamParam, dUimage);
	double dZcameraRoad = calcZcameraFromYcameraOnRoad(pCamParam, dYcamera);
	//heryms set this function
	double dZvehicle = dZcameraRoad + pCamParam->cam_pos_z;	// 
	return dZvehicle;
}
double CoordTransform::calcXvehicleFromVimageAndZvehicle(CamParam *pCamParam, double dVimage, double dZvehicle)
{
	double dZcamera = dZvehicle - pCamParam->cam_pos_z;
	double dXcamera = calcXcameraFromVimage(pCamParam, dVimage);
	
	double dXcameraRoad = dXcamera * dZcamera / pCamParam->f;
	double dXvehicle = dXcameraRoad + (pCamParam->cam_pos_x);;	// (lateral)
	return dXvehicle;
}
double CoordTransform::calcYcameraFromUimage(CamParam *pCamParam, double dUimage)
{
	double dYcamera = dUimage - pCamParam->fy * (pCamParam->pitch);
	return dYcamera;
}
// Vimage->Xcamera
double CoordTransform::calcXcameraFromVimage(CamParam *pCamParam, double dVimage)
{
	double dXcamera = dVimage + pCamParam->f * pCamParam->yaw;
	return dXcamera;
}
int CoordTransform::ImageToRoad(CamParam *pCamParam, int iIimage, int iJimage, double *pdIRoad, double *pdJRoad){
	// convert image coordinate to center in pixel center
	double dUimage = (double)iIimage - pCamParam->cy;
	double dVimage = (double)iJimage - pCamParam->cx;
	double dZvehicle = calcZvehicleFromUimage(pCamParam, dUimage);
	double dXvehicle = calcXvehicleFromVimageAndZvehicle(pCamParam, dVimage, dZvehicle);
	*pdIRoad = dZvehicle;
	*pdJRoad = dXvehicle;
	return 1;

}
int CoordTransform::calciIsrcFromZcameraOnRoad(CamParam *pCamParam, double dZcamera){
	double dYcamera = -pCamParam->fy * (pCamParam->cam_pos_y) / dZcamera;
	double dUimage = dYcamera + pCamParam->fy * (pCamParam->pitch);
	int iIsrc = (int)(dUimage - 0.5 + pCamParam->cy);
	return iIsrc;

}
// //(longitidinal,lateral->horizontal)	// Xvehicle,Zvehicle->Vimage
int CoordTransform::calciJsrcFromXvehicleAndZvehicleOnRoad(CamParam *pCamParam, double dXvehicleOnRoad, double dZvehicleOnRoad)
{
	double dXcameraOnRoad = dXvehicleOnRoad - pCamParam->cam_pos_x;
	double dZcameraOnRoad = dZvehicleOnRoad - pCamParam->cam_pos_z;
	double dVimage = calcVimageFromXcameraAndZcameraOnRoad(pCamParam, dXcameraOnRoad, dZcameraOnRoad);
	int iJsrc = (int)(dVimage - 0.5 + pCamParam->cx);	
	return iJsrc;
	
}
double CoordTransform::calcVimageFromXcameraAndZcameraOnRoad(CamParam *pCamParam, double dXcameraRoad, double dZcameraRoad)
{
	double dVimage = dXcameraRoad * pCamParam->f / dZcameraRoad - pCamParam->f * pCamParam->yaw;
	return dVimage;
}
#pragma endregion IMAGE_TO_ROAD_AND_ROAD_TO_IMAGE


//转换全局世界坐标到局部坐标
/************************************************************************/
/*
Param:
org 目标坐标系的原点和方位角
xIn 输入的X坐标
yIn 输入的Y坐标
xOut 输出的X坐标
yOut  输出的Y坐标
Function：
将全局坐标投影到车体（或者指定位置）的局部坐标系,全局坐标的X和Y与局部坐标是相反的
*/
/************************************************************************/
int CoordTransform::WorldtoLocal(PosPoint org, double xIn, double yIn, double &xOut, double &yOut)
{
	double dx = 0, dy = 0, dstX = 0, dstY = 0;
	dx = xIn - org.X;
	dy = yIn - org.Y;
	dstX = dx*cos(org.Angle) - dy*sin(org.Angle);
	dstY = dx*sin(org.Angle) + dy*cos(org.Angle);
	xOut = dstX;
	yOut = dstY;

	return 1;
}

//转换局部坐标到全局世界坐标
int CoordTransform::LocaltoWorld(PosPoint org, double xIn, double yIn, double &xOut, double &yOut)
{
	double dx = 0, dy = 0, dstX = 0, dstY = 0;
	dstX = xIn*cos(org.Angle) + yIn*sin(org.Angle);
	dstY = -xIn*sin(org.Angle) + yIn*cos(org.Angle);
	xOut = dstX + org.X;
	yOut = dstY + org.Y;
	return 1;
}
