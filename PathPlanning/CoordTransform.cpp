#include "CoordTransform.h"
#include "Topology.h"
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


void CoordTransform::WorldToLocal(PosPoint org, PosPoint ptIn, PosPoint* ptOut)
{
	ptOut->x = ptIn.x - org.x;
	ptOut->y = ptIn.y - org.y;
	Topology::Rotate(PI / 2 - org.angle, ptOut->x, ptOut->y, ptOut->x, ptOut->y);
	ptOut->angle = PI / 2 - org.angle + ptIn.angle;
}

void CoordTransform::LocalToWorld(PosPoint org, PosPoint ptIn, PosPoint* ptOut)
{
	Topology::Rotate(org.angle - PI / 2, ptIn.x, ptIn.y, ptOut->x, ptOut->y);
	ptOut->x += org.x;
	ptOut->y += org.y;
	ptOut->angle = -ptIn.angle + org.angle + PI / 2;
}
double CoordTransform::TrimLocalPathToCurPt(std::vector<RoadPoint>& localPath)
{
	double min = DBL_MAX;
	int minIndex = 0;
	for (int i = 0; i < localPath.size(); i++)
	{
		PosPoint pt = localPath[i];
		double dis = pt.x*pt.x + pt.y*pt.y;
		if (min > dis&&pt.angle > 0) {
			min = dis;
			minIndex = i;
		}
	}
	std::pair<double, double> v1;
	v1.first = localPath[minIndex + 1].x - localPath[minIndex].x;
	v1.second = localPath[minIndex + 1].y - localPath[minIndex].y;
	RoadPoint firstPt;
	firstPt.y = (-v1.first * v1.second * localPath[minIndex].x
		+ v1.first * v1.first * localPath[minIndex].y)
		/ (v1.first * v1.first + v1.second * v1.second);
	firstPt.x = (-v1.second * firstPt.y) / (v1.first);
	firstPt.angle = atan2(v1.second, v1.first);
	localPath.erase(localPath.begin(), localPath.begin() + minIndex);
	localPath[0] = firstPt;
	double qi = sqrt(firstPt.x * firstPt.x + firstPt.y * firstPt.y);
	if (firstPt.x < 0)
	{
		return -qi;
	}
	return qi;
}
//
////转换全局世界坐标到局部坐标
///************************************************************************/
///*
//Param:
//org 目标坐标系的原点和方位角
//xIn 输入的X坐标
//yIn 输入的Y坐标
//xOut 输出的X坐标
//yOut  输出的Y坐标
//Function：
//将全局坐标投影到车体（或者指定位置）的局部坐标系,全局坐标的X和Y与局部坐标是相反的
//*/
///************************************************************************/
//int CoordTransform::WorldtoLocal(PosPoint org, double xIn, double yIn, double &xOut, double &yOut)
//{
//	double dx = 0, dy = 0, dstX = 0, dstY = 0;
//	dx = xIn - org.x;
//	dy = yIn - org.y;
//	dstX = dx*cos(org.angle) - dy*sin(org.angle);
//	dstY = dx*sin(org.angle) + dy*cos(org.angle);
//	xOut = dstX;
//	yOut = dstY;
//
//	return 1;
//}
//
////转换局部坐标到全局世界坐标
//int CoordTransform::LocaltoWorld(PosPoint org, double xIn, double yIn, double &xOut, double &yOut)
//{
//	double dx = 0, dy = 0, dstX = 0, dstY = 0;
//	dstX = xIn*cos(org.angle) + yIn*sin(org.angle);
//	dstY = -xIn*sin(org.angle) + yIn*cos(org.angle);
//	xOut = dstX + org.x;
//	yOut = dstY + org.y;
//	return 1;
//}

bool CoordTransform::LocaltoGrid(PosPoint org, int &xOut, int &yOut)
{
	xOut = (int)((org.x - X_START) / Grid);
	yOut = (int)((org.y - Y_START) / Grid);
	if (xOut>=0 && xOut <= Grid_NUM_X && yOut >=0 && yOut<=Grid_NUM_Y)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool CoordTransform::GridtoLocal(int xIn, int yIn, double &xOut, double &yOut){
	xOut = xIn * Grid;
	yOut = yIn * Grid;
	if (xOut >= X_START && xOut <= MAP_WIDTH && yOut >= Y_START && yOut <= MAP_HEIGHT)
	{
		return true;
	}
	else{
		return false;
	}
}

//x1.y==x2.y
std::vector<int> CoordTransform::XTriangleToGrids(PosPoint x1, PosPoint x2, PosPoint y, PosPoint yGrid) {
	if (x1.x > x2.x) {
		PosPoint tmp=x1;
		x1 = x2;
		x2 = tmp;
	}
	double len1=fabs(y.x-x1.x);
	double len2=fabs(y.x-x2.x);
	double height=fabs(y.y-x1.y);
	std::vector<int> grids;
	int xindex = yGrid.x;
	int yindex = yGrid.y;
	for (int i = 0;; i++, (y.y>x1.y)?(yindex--):(yindex++)) {
		double tmpH = Grid*(i + 1);
		if (tmpH > fabs(height)) break;
		double tmpX1 = len1*tmpH / height;
		double tmpX2 = len2*tmpH / height;
		int x1GridNum = tmpX1 / Grid + 0.5;
		int x2GridNum = tmpX2 / Grid + 0.5;
		for (int n = 1; n <= x1GridNum; n++) {
			int xi = xindex - n;
			int index = yindex*MAP_WIDTH + xindex;
			if (index>0&&index < MAP_WIDTH*MAP_HEIGHT){
				grids.push_back(index);
			}
		}
		for (int n = 0; n < x2GridNum; n++) {
			int xi = xindex + n;
			int index = yindex*MAP_WIDTH + xindex;
			if( index>0&&index < MAP_WIDTH*MAP_HEIGHT){
				grids.push_back(index);
			}
		}

	}
	return grids;
}