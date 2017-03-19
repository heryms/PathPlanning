#ifndef __LCMHANDLER
#define __LCMHANDLER
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>
#include <chrono>
template <typename T> class LcmHandler 
{
protected:
	bool recvOn;
private:
	void *  _lpParam;
	typedef void (*msg_output)(const T *msg, void * lpParam);
	msg_output _rcvMsg; //callback function for output data
	std::string _channel;  //channel  id
	lcm::LCM *_lcm;
public:

	~LcmHandler() {
			delete _lcm;
	}
	LcmHandler()
	{
		_lpParam = nullptr;
		_channel = "";
		_rcvMsg = nullptr;
		_lcm = nullptr;
		recvOn = false;
	}

	static void recvThread(LcmHandler<T>* h) {
		while (h->recvOn) {
			h->grabTimeout(20);
		}
	}

	void handleMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const T * msg)
	{
		_rcvMsg(msg,_lpParam);
	}
	int intialSend(std::string net, std::string channel)
	{
		_channel = channel;
		delete _lcm;
		_lcm = new lcm::LCM(net);
		if (!_lcm->good())
		{
			return 0;
		}
		return 1;
	}
	void sendLcm(T* lane)
	{
		_lcm->publish(_channel, lane);
	}
	//0 lcm²»³É¹¦
	//int initialLcm(std::string net, std::string channel, msg_output rcv)
	//{
	//	_rcvMsg = rcv;
	//	_channel = channel;
	//	delete _lcm;
	//	_lcm = new lcm::LCM(net);
	//	if (!_lcm->good())
	//	{
	//		return 0;
	//	}
	//	_lcm->subscribe(channel, &LcmHandler::handleMessage, this);
	//	return 1;
	//}

	int initialLcm(std::string net, std::string channel, msg_output rcv, void * lpParam)
	{
		_lpParam = lpParam;
		_rcvMsg = rcv;
		_channel = channel;
		delete _lcm;
		_lcm = new lcm::LCM(net);
		if (!_lcm->good())
		{
			return 0;
		}
		_lcm->subscribe(channel, &LcmHandler::handleMessage, this);
		recvOn = true;
		std::thread t(recvThread, this);
		t.detach();
		return 1;
	}

	void uninitialLcm() {
		recvOn = false;
		//std::this_thread::sleep_for(std::chrono::milliseconds(20));
		delete _lcm;
		_channel = " ";
		_rcvMsg = NULL;
		_lcm = nullptr;
		_lpParam = nullptr;
	}

	int grab()
	{
		//_beginthread(listen,0,this);
		return _lcm->handle();
	}

	int grabTimeout(int millseconds) {
		return _lcm->handleTimeout(millseconds);
	}

};
#endif // !__LCMHANDLER