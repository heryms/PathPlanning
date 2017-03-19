#pragma once
#ifndef __LOOP_THREAD_H
#define __LOOP_THREAD_H

#include <condition_variable>
#include <mutex>
#include <thread>

#define __thread_sleep_for(ms) {std::this_thread::sleep_for(std::chrono::milliseconds(ms));}

class ZBaseLoopThread
{
public:
	ZBaseLoopThread();
	ZBaseLoopThread(std::function<bool()> funcRunLoop,
		std::function<bool()> funcBeforeLoop = []()->bool{ return true; },
		std::function<int()> funcAfterLoop = []()->int {return 0; });
	ZBaseLoopThread(const ZBaseLoopThread&);
	ZBaseLoopThread& operator=(const ZBaseLoopThread&);
	virtual ~ZBaseLoopThread();
	void Start();
	bool Stop(int milliseconds = -1);
	bool Wait(int milliseconds = -1);
	void Suspend();
	void Resume();
	bool Started();
	bool Ended();
	bool Suspended();
	void SetLoopInterval(int milliseconds);
private:
	std::function<bool()> funcBeforeLoop;
	std::function<bool()> funcRunLoop;
	std::function<int()> funcAfterLoop;
protected:
	int loopInterval;
	bool loopOn;
	bool suspendOn;
	bool threadStart;
	bool threadEnd;
	bool realInSuspend;
	std::mutex endmtx;
	std::condition_variable endcv;
	std::mutex suspendmtx;
	std::condition_variable suspendcv;
	std::thread t;
	static int ThreadBody(ZBaseLoopThread* loopThread);
	/**
	* @return true enter the loop, false exit thread.
	**/
	virtual bool BeforeLoop();
	/**
	* @return the value returned while thread ended.
	**/
	virtual int AfterLoop();
	/**
	* The function running in loop.
	* @return true continue loop, false exit loop.
	**/
	virtual bool RunLoop();
};


class ZLoopThreadBuilder {
private:
	std::function<bool()> funcBeforeLoop;
	std::function<bool()> funcRunLoop;
	std::function<int()> funcAfterLoop;
public:
	ZLoopThreadBuilder();
	void setBeforeLoop(std::function<bool()>);
	void setRunLoop(std::function<bool()>);
	void setAfterLoop(std::function<int()>);
	ZBaseLoopThread build();
};
#endif