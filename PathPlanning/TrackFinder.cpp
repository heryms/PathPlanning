#include "TrackFinder.h"
#include "Topology.h"


TrackFinder::TrackFinder()
{
}


TrackFinder::~TrackFinder()
{
}

int TrackFinder::FindPointIndex(std::vector<RoadPoint>& path, RoadPoint curX)
{
	if (path.size() <= 1) {
		return -1;
	}
	std::vector<int> point;
	double dis = 1;
	while (point.empty())
	{
		for (int i = 0; i < path.size(); i++) {
			if (Topology::Distance2(path[i], curX) < dis*dis) {
				if (cos(path[i].angle-curX.angle) > 0)
				{
					point.push_back(i);
				}
			}
		}
		dis += 1;
		if (dis > 10) {
			return -1;
		}
	}
	int minIndex = point[0];
	double min = Topology::Distance2(path[0], curX);
	for (int index : point) {
		double t = Topology::Distance2(path[index], curX);
		if (t < min) {
			min = t;
			minIndex = index;
		}
	}
	return minIndex;
}

int TrackFinder::AnchorPoint(std::vector<RoadPoint>& path, RoadPoint curX, int curIndex, double distance)
{
	if (curIndex < 0) {
		curIndex = FindPointIndex(path, curX);
	}
	if (curIndex < 0) {
		return  -1;
	}
	if (curIndex >= path.size()) {
		return -1;
	}
	distance /= 0.2;
	double dis = sqrt(Topology::Distance2(path[curIndex], curX));
	for (int i = curIndex + 1; i < path.size(); i++) {
		dis += sqrt(Topology::Distance2(path[i], path[i - 1]));
		if (distance < dis) {
			return i;
		}
	}
	return path.size() - 1;
}

bool TrackFinder::InCurve(bool isInCurve, std::vector<RoadPoint>& path, RoadPoint curX, int curIndex)
{
	double anchorDis = 20 / 0.2;
	int anchorIndex = AnchorPoint(path, curX, curIndex, anchorDis);
	if(anchorIndex<0){
		return isInCurve;
	}
	if (Topology::Distance2(path[anchorIndex], curX)<(isInCurve ? 0.999 : 0.97)*anchorDis*anchorDis) {
		return true;
	}
	else {
		return false;
	}
}
