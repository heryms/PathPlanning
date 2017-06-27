#include "PureTrack.h"
#include "TrackFinder.h"
#include "LocalCarStatus.h"
#include "Topology.h"
#include "CarControl.h"
PureTrack::PureTrack()
{
}


PureTrack::~PureTrack()
{
}


void PureTrack::Track()
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
		{
			RoadPoint refX;
			if (TrackFinder::FindPursuitPoint(path, curX, refX, LocalCarStatus::GetInstance().GetSpeed()/3.6)){
				info.steerAngle = TrackFinder::MiddlePiont_Shan(curX,refX.x, refX.y);
				info.gear = D;
				info.speed = 10;
				info.state = START;
			}
			else{
				info.gear = D;
				info.speed = 0;
				info.state = E_STOP;
				info.steerAngle = LocalCarStatus::GetInstance().GetSteerAngle();
			}
			break;
		}
		inCurve = TrackFinder::InCurve(inCurve, path, curX, curIndex);
		RoadPoint refX;
		if (inCurve) {
			refX = path[TrackFinder::AnchorPoint(path, curX, curIndex, 6.5)];
			info.speed = refSpeedCurve;
		}
		else {
			refX = path[TrackFinder::AnchorPoint(path, curX, curIndex, 12)];
			info.speed = refSpeedStraight;
		}
		info.gear = D;
		info.state = START;
		RadAngle alpha = curX.angle - (RadAngle)atan2(refX.y - curX.y, refX.x - curX.x);
	//	std::cout << "alpha:" + std::to_string(alpha)<< std::endl;
		double ld = sqrt(Topology::Distance2(refX, curX));
		info.steerAngle =
			atan(2 * LocalCarStatus::GetInstance().GetL()*sin(alpha) / ld)
			*LocalCarStatus::GetInstance().GetSteerRatio()*180.0 / PI;
	} while (false);
	CarControl::GetInstance().SendCommand(info);
}
