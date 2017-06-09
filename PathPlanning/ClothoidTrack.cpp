#include "ClothoidTrack.h"
#include "TrackFinder.h"
#include "LocalCarStatus.h"
#include "Topology.h"
#include "CarControl.h"
#include "Clothoid.h"



ClothoidTrack::ClothoidTrack()
{
}


ClothoidTrack::~ClothoidTrack()
{
}

void ClothoidTrack::Track()
{
	if (path.size() < 10) return;
	CarInfo info;
	RoadPoint curX = RoadPoint::toRoadPoint(LocalCarStatus::GetInstance().GetPosition());
	int curIndex = TrackFinder::FindPointIndex(path, curX);
	do {
		if (curIndex < 0) {
			info.gear = D;
			info.speed = 0;
			info.state = E_STOP;
			info.steerAngle = LocalCarStatus::GetInstance().GetSteerAngle();
			break;
		}
		inCurve = TrackFinder::InCurve(inCurve, path, curX, curIndex);
		RoadPoint refX;
		if (inCurve) {
			refX = path[TrackFinder::AnchorPointStraight(path, curX, curIndex, 12)];
			info.speed = refSpeedCurve;
		}
		else {
			refX = path[TrackFinder::AnchorPointStraight(path, curX, curIndex, 12)];
			info.speed = refSpeedStraight;
		}
		info.gear = D;
		info.state = START;
		Clothoid clothoid(curX.x, curX.y, curX.angle, refX.x, refX.y, refX.angle);
		//std::vector<RoadPoint> rdpt;
		//rdpt.resize(100);
		//clothoid.PointsOnClothoid(rdpt, 100);
		info.steerAngle = atan(-clothoid.getK()*LocalCarStatus::GetInstance().GetL())
			*LocalCarStatus::GetInstance().GetSteerRatio()
			* 180 / PI;
	} while (false);
	CarControl::GetInstance().SendCommand(info);
}
