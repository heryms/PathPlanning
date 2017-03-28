#include "ClothoidTrack.h"
#include "LocalCarStatus.h"
#include "LoopThread.h"
#include "CarControl.h"
#include <cmath>

class TrackThread : public ZBaseLoopThread{
private:
	ClothoidTrack * track;
	int lastIndex = 0;
public:
	TrackThread(void* parent):
		track((ClothoidTrack*)parent)
	{
		loopInterval = 10;
	}
	
	bool RunLoop() {
		std::unique_lock<std::mutex> lk(track->m_lock);
		if (track->path.size() <= 10) return true;
		track->Track(FindCurv(LocalCarStatus::GetInstance().GetPosition()));
		return true;
	}

	double FindCurv(PosPoint cur) {
		double min = cur.x*track->path[lastIndex].x + cur.y*track->path[lastIndex].y;
		int nowIndex=lastIndex;
		for (int i = lastIndex + 1; i < track->path.size(); i++) {
			double dis = cur.x*track->path[i].x + cur.y*track->path[i].y;
			if (dis > min) {
				nowIndex = i;
				break;
			}
		}
		if (nowIndex == 0) {
			lastIndex = 0;
			return track->path[nowIndex].k;
		}
		else {
			lastIndex = nowIndex - 1;
			double dis1 = cur.x*track->path[nowIndex].x + cur.y*track->path[nowIndex].y;
			double dis2 = cur.x*track->path[nowIndex - 1].x + cur.y*track->path[nowIndex - 1].y;
			double dk = track->path[nowIndex].k - track->path[nowIndex - 1].k;
			return dk*dis2 / (dis1 + dis2) + track->path[nowIndex - 1].k;
		}
	}

	void Reset() {
		lastIndex = 0;
	}
};


ClothoidTrack::ClothoidTrack()
{
	thread = new TrackThread(this);
}


ClothoidTrack::~ClothoidTrack()
{
	delete thread;
	thread = nullptr;
}

void ClothoidTrack::SetPath(std::vector<RoadPoint> path)
{
	std::unique_lock<std::mutex> lk(m_lock);
	this->path = path;
	PosPoint p;
	p.x = 0;
	p.y = 0;
	p.angle = PI / 2;
	LocalCarStatus::GetInstance().SetCurPosition(p);
	thread->Reset();
}

void ClothoidTrack::Track(double k)
{
	CarInfo info;
	info.gear = D;
	info.state = START;
	info.speed = 5;
	info.steerAngle = atan(-k*2.34) * 18 * 180 / PI-8;
	CarControl::GetInstance().SendCommand(info);
}

void ClothoidTrack::Start()
{
	PosPoint p;
	p.x = 0;
	p.y = 0;
	p.angle = PI / 2;
	LocalCarStatus::GetInstance().SetCurPosition(p);
	LocalCarStatus::GetInstance().Start();
	thread->Start();

}

void ClothoidTrack::Suspend()
{
	thread->Suspend();
	LocalCarStatus::GetInstance().Suspend();
}

void ClothoidTrack::Resume()
{
	thread->Resume();
	LocalCarStatus::GetInstance().Resume();
}

void ClothoidTrack::End()
{
	thread->Stop();
	LocalCarStatus::GetInstance().End();
}
