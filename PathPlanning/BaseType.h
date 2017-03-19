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
	double  X;
	double  Y;
	RadAngle  Angle;
}PosPoint;

typedef struct {
	double speed;
	double steerAngle;
}CarInfo;


#endif // !_PATH_PLANNING_BASE_TYPE_H
