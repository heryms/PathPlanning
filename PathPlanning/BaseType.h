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


enum EGear {
	UNKNOWN = 0,
	N = 1,
	D = 2,
	R = 3,
	P = 4,
	L = 5
};

enum ERunState {
	IDLE=0,
	START=1,
	STOP=2,
	E_STOP=3
};

typedef struct {
	double speed;
	double steerAngle;
	EGear gear;
	ERunState state;
}CarInfo;

struct PosPoint
{
	double  x;
	double  y;
	RadAngle  angle;

};

struct RoadPoint : public PosPoint
{
	//double x;
	//double y;
	//double angle;
	double changeangle;
	double k;

	static RoadPoint toRoadPoint(PosPoint p) {
		RoadPoint rp;
		rp.x = p.x;
		rp.y = p.y;
		rp.angle = p.angle;
		return rp;
	}
};

struct LINESEG
{
	RoadPoint s;
	RoadPoint e;

};
#endif // !_PATH_PLANNING_BASE_TYPE_H
