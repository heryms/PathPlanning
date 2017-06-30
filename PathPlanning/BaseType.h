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
	EL = 5
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

struct PointPt
{
public:
	double  x;
	double  y;
	PointPt() :x(0), y(0) {

	}
};

struct PosPoint :public PointPt
{
public:
	RadAngle  angle;
	PosPoint() :PointPt(), angle(0) {

	}
	PosPoint(const PointPt& p) :PointPt(p),angle(0) {
	}

	PosPoint &operator =(const PointPt& p) {
		this->x = p.x;
		this->y = p.y;
		return *this;
	}
};

struct RoadPoint : public PosPoint
{
public:
	//double x;
	//double y;
	//double angle;
	double changeangle;
	double k;
	RoadPoint():PosPoint(),changeangle(0),k(0) {

	}

	RoadPoint(const PosPoint& p):PosPoint(p), changeangle(0), k(0) {
	}

	RoadPoint &operator =(const PosPoint& p) {
		this->x = p.x;
		this->y = p.y;
		this->angle = p.angle;
		return *this;
	}
};

struct LINESEG
{
	RoadPoint s;
	RoadPoint e;

};
#endif // !_PATH_PLANNING_BASE_TYPE_H
