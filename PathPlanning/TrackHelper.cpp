#include "TrackHelper.h"
#include "LocalCarStatus.h"
#include "LoopThread.h"
#include <cmath>
#include "BaseTrack.h"
#include "MPCTrack.h"
#include "ClothoidTrack.h"
#include "PureTrack.h"
#include "DataCenter.h"

class TrackThread : public ZBaseLoopThread{
private:
	TrackHelper * track;
	//int lastIndex = 0;
public:
	TrackThread(void* parent):
		track((TrackHelper*)parent)
	{
		loopInterval = 100;
	}
	
	bool RunLoop() {
		track->Track();
		return true;
	}

	//double FindCurv(PosPoint cur) {
	//	double min = cur.x*track->path[lastIndex].x + cur.y*track->path[lastIndex].y;
	//	int nowIndex=lastIndex;
	//	for (int i = lastIndex + 1; i < track->path.size(); i++) {
	//		double dis = cur.x*track->path[i].x + cur.y*track->path[i].y;
	//		if (dis > min) {
	//			nowIndex = i;
	//			break;
	//		}
	//	}
	//	if (nowIndex == 0) {
	//		lastIndex = 0;
	//		return track->path[nowIndex].k;
	//	}
	//	else {
	//		lastIndex = nowIndex - 1;
	//		double dis1 = cur.x*track->path[nowIndex].x + cur.y*track->path[nowIndex].y;
	//		double dis2 = cur.x*track->path[nowIndex - 1].x + cur.y*track->path[nowIndex - 1].y;
	//		double dk = track->path[nowIndex].k - track->path[nowIndex - 1].k;
	//		return dk*dis2 / (dis1 + dis2) + track->path[nowIndex - 1].k;
	//	}
	//}

	void Reset() {
		//lastIndex = 0;
	}
};


TrackHelper::TrackHelper()
{
	track = new PureTrack();
	//track = new ClothoidTrack();
	//track = new MPCTrack();
	thread = new TrackThread(this);
}


TrackHelper::~TrackHelper()
{
	delete thread;
	thread = nullptr;
	delete track;
	track = nullptr;
}

void TrackHelper::SetLocalPath(std::vector<RoadPoint>& path)
{
	//std::unique_lock<std::mutex> lk(m_lock);
	thread->Suspend();
	LocalCarStatus::GetInstance().Suspend();
	//this->path = path;

	PosPoint p;
	p.x = 0 ;
	p.y = 0;
	p.angle = PI / 2;
	thread->Reset();
	LocalCarStatus::GetInstance().SetOriginPos(DataCenter::GetInstance().GetCurPosition());
	LocalCarStatus::GetInstance().SetCurPosition(p);
	LocalCarStatus::GetInstance().Resume();
	track->SetLocalPath(path);
	thread->Resume();
}

void TrackHelper::SetPath(std::vector<RoadPoint>& path)
{
	thread->Suspend();
	LocalCarStatus::GetInstance().Suspend();
	PosPoint p = DataCenter::GetInstance().GetCurPosition();
	thread->Reset();
	LocalCarStatus::GetInstance().SetOriginPos(p);
	LocalCarStatus::GetInstance().SetCurPosition(p);
	LocalCarStatus::GetInstance().Resume();
	track->SetLocalPath(path);
	thread->Resume();
}

void TrackHelper::Track()
{
	track->Track();
}

void TrackHelper::Start()
{
	PosPoint p;
	p.x = 0;
	p.y = 0;
	p.angle = PI / 2;
	LocalCarStatus::GetInstance().SetCurPosition(p);
	LocalCarStatus::GetInstance().Start();
	thread->Start();

}

void TrackHelper::Suspend()
{
	thread->Suspend();
	LocalCarStatus::GetInstance().Suspend();
}

void TrackHelper::Resume()
{
	thread->Resume();
	LocalCarStatus::GetInstance().Resume();
}

void TrackHelper::End()
{
	thread->Stop();
	LocalCarStatus::GetInstance().End();
}
