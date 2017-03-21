#ifndef _PATH_PLANNING_BASE_TYPE_H
#define _PATH_PLANNING_BASE_TYPE_H
#include "BaseAngle.h"
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
	double  x;
	double  y;
	RadAngle  angle;
}PosPoint;

typedef struct {
	double speed;
	double steerAngle;
}CarInfo;

typedef struct tagRoadPoint : public PosPoint
{
	//double x;
	//double y;
	//double angle;
	double changeangle;
	double k;
}RoadPoint;

struct LINESEG
{
	RoadPoint s;
	RoadPoint e;

};
#endif // !_PATH_PLANNING_BASE_TYPE_H
