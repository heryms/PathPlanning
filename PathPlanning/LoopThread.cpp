
#include "LoopThread.h"

ZBaseLoopThread::ZBaseLoopThread() {
	loopOn = false;
	suspendOn = false;
	threadEnd = false;
	threadStart = false;
	loopInterval = 1;
	this->funcBeforeLoop = std::bind(&ZBaseLoopThread::BeforeLoop,this);
	this->funcRunLoop = std::bind(&ZBaseLoopThread::RunLoop, this);
	this->funcAfterLoop = std::bind(&ZBaseLoopThread::AfterLoop, this);
}

ZBaseLoopThread::ZBaseLoopThread(std::function<bool()> funcRunLoop, std::function<bool()> funcBeforeLoop, std::function<int()> funcAfterLoop)
{
	loopOn = false;
	suspendOn = false;
	threadEnd = false;
	threadStart = false;
	loopInterval = 1;
	this->funcBeforeLoop = funcBeforeLoop;
	this->funcRunLoop = funcRunLoop;
	this->funcAfterLoop = funcAfterLoop;
}

ZBaseLoopThread::ZBaseLoopThread(const ZBaseLoopThread & th)
{
	this->funcAfterLoop = th.funcAfterLoop;
	this->funcBeforeLoop = th.funcBeforeLoop;
	this->funcRunLoop = th.funcRunLoop;
	this->loopInterval = th.loopInterval;
	loopOn = false;
	suspendOn = false;
	threadEnd = false;
	threadStart = false;
}

ZBaseLoopThread & ZBaseLoopThread::operator=(const ZBaseLoopThread & th)
{
	this->funcAfterLoop = th.funcAfterLoop;
	this->funcBeforeLoop = th.funcBeforeLoop;
	this->funcRunLoop = th.funcRunLoop;
	this->loopInterval = th.loopInterval;
	loopOn = false;
	suspendOn = false;
	threadEnd = false;
	threadStart = false;
	return *this;
}

ZBaseLoopThread::~ZBaseLoopThread()
{
	if (t.joinable()) {
		t.detach();
	}
}

void ZBaseLoopThread::Start()
{
	if (loopOn) {
		return;
	}
	if (t.joinable()) {
		t.join();
	}
	loopOn = true;
	suspendOn = false;
	t = std::thread(ThreadBody, this);
}

bool ZBaseLoopThread::Stop(int milliseconds)
{
	if (threadStart && !loopOn) {
		return false;
	}
	else if (!threadStart) {
		return true;
	}
	loopOn = false;
	if (milliseconds < 0) {
		t.join();
		return true;
	}
	else {
		std::unique_lock <std::mutex> lck(endmtx);
		bool succeed = (endcv.wait_for(lck, std::chrono::milliseconds(milliseconds), [=] {return threadEnd; }));
		if (succeed) {
			t.detach();
		}
		return succeed;
	}
}

bool ZBaseLoopThread::Wait(int milliseconds)
{
	return Stop(milliseconds);
}

void ZBaseLoopThread::Suspend()
{
	if (!threadStart) {
		return;
	}
	if (suspendOn) {
		return;
	}
	realInSuspend = false;
	suspendOn = true;
	std::unique_lock<std::mutex> lck(suspendmtx);
	suspendcv.wait(lck, [=] {return realInSuspend; });
}

void ZBaseLoopThread::Resume()
{
	suspendOn = false;
}

bool ZBaseLoopThread::Started()
{
	return threadStart;
}

bool ZBaseLoopThread::Ended()
{
	return !threadStart;
}

bool ZBaseLoopThread::Suspended()
{
	return loopOn&&suspendOn;
}

void ZBaseLoopThread::SetLoopInterval(int milliseconds)
{
	loopInterval = milliseconds;
}

int ZBaseLoopThread::ThreadBody(ZBaseLoopThread * loopThread)
{
	loopThread->threadStart = true;
	loopThread->threadEnd = false;
	if (!loopThread->funcBeforeLoop()) {
		loopThread->loopOn = false;
		loopThread->threadEnd = true;
		loopThread->threadStart = false;
		return 0;
	}
	while (loopThread->loopOn)
	{
		if (loopThread->suspendOn) {
			if (!loopThread->realInSuspend) {
				std::unique_lock<std::mutex> lck(loopThread->suspendmtx);
				loopThread->realInSuspend = true;
				lck.unlock();
				loopThread->suspendcv.notify_one();
			}
			//std::this_thread::yield();
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		if (!loopThread->funcRunLoop()) {
			loopThread->loopOn = false;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(loopThread->loopInterval));
	}
	int ret = loopThread->funcAfterLoop();
	std::unique_lock <std::mutex> lck(loopThread->endmtx);
	loopThread->threadEnd = true;
	lck.unlock();
	loopThread->endcv.notify_one();
	loopThread->threadStart = false;
	return ret;
}

bool ZBaseLoopThread::BeforeLoop()
{
	return true;
}

int ZBaseLoopThread::AfterLoop()
{
	return 0;
}

bool ZBaseLoopThread::RunLoop()
{
	return true;
}

ZLoopThreadBuilder::ZLoopThreadBuilder()
{
	funcBeforeLoop = []()->bool {return true; };
	funcAfterLoop = []()->int {return 0; };
	funcRunLoop = []()->bool {return true; };
}

void ZLoopThreadBuilder::setBeforeLoop(std::function<bool()> beforeloop)
{
	funcBeforeLoop = beforeloop;
}

void ZLoopThreadBuilder::setRunLoop(std::function<bool()> runnable)
{
	funcRunLoop = runnable;
}

void ZLoopThreadBuilder::setAfterLoop(std::function<int()> afterloop)
{
	funcAfterLoop = afterloop;
}

ZBaseLoopThread ZLoopThreadBuilder::build()
{
	return ZBaseLoopThread(funcRunLoop, funcBeforeLoop, funcAfterLoop);
}
