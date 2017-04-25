#include "BaseTrack.h"



BaseTrack::BaseTrack()
{
}


BaseTrack::~BaseTrack()
{
}

void BaseTrack::SetPath(std::vector<RoadPoint>& path)
{
	this->path = path;// std::move(path);
}

void BaseTrack::SetRefSpeed(double straight, double curve)
{
	this->refSpeedStraight = straight;
	this->refSpeedCurve = curve;
}
