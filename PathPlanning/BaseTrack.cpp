#include "BaseTrack.h"



BaseTrack::BaseTrack()
{
	b_stop = false;
	recommendSpeed = -1.0;
}


BaseTrack::~BaseTrack()
{
}

void BaseTrack::SetLocalPath(std::vector<RoadPoint>& path)
{
	this->path = path;// std::move(path);
}

void BaseTrack::SetRefSpeed(double straight, double curve)
{
	this->refSpeedStraight = straight;
	this->refSpeedCurve = curve;
}
