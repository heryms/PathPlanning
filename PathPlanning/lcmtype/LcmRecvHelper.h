#include <iostream>
#include "LcmType_Handler.h"

using std::chrono::steady_clock;

template<typename T>
class LcmRecvHelper : public LcmHandler<T> {
private:
	steady_clock::time_point startTime;
	steady_clock::time_point endTime;
	bool hasMsg;
public:
	LcmRecvHelper() :LcmHandler<T>(), hasMsg(false) {

	}

	bool HasLcmMessage() {
		return hasMsg;
	}

	int initialLcm(std::string net, std::string channel, msg_output rcv, void * lpParam) {
		std::cout << "Start " << typeid(T).name() << "!" << std::endl;
		int ret = LcmHandler<T>::initialLcm(net, channel, rcv, lpParam) > 0;
		if (ret > 0) {
			return 1;
		}
		else if(ret < 0) {
			std::cout << "Warning: May have started " << typeid(T).name() << "!" << std::endl;
			return -1;
		}
		else {
			std::cout << "Error: Could not start " << typeid(T).name() << "!" << std::endl;
			return 0;
		}
	}

	bool RecvBody(){
		if (grabTimeout(20) > 0){
			startTime = steady_clock::now();
		}
		endTime = steady_clock::now();
		std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		if (time.count() > 500) {
			if (hasMsg) {
				hasMsg = false;
				std::cout << "Warning::May not " << typeid(T).name() << "!" << std::endl;
			}
		}
		else {
			if (!hasMsg) {
				hasMsg = true;
				std::cout << "Received " << typeid(T).name() << "!" << std::endl;
			}
		}
		return true;
	}

	//static void recvThread(LcmHandler<T>* h) {
	//	startTime = endTime = steady_clock::now();
	//	while (h->recvOn) {
	//		if (h->grabTimeout(20) > 0){
	//			startTime = steady_clock::now();
	//		}
	//		endTime = steady_clock::now();
	//		std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	//		if (time.count() > 500){
	//			hasMsg = false;
	//			std::cout << "Warning::May not" << T.getTypeName() << "!" << std::endl;
	//		}
	//		else{
	//			hasMsg = true;
	//		}
	//	}
	//}

	void uninitialLcm(){
		std::cout << typeid(T).name() << " Stoping..." << std::endl;
		LcmHandler<T>::uninitialLcm();
		hasMsg = false;
		std::cout << typeid(T).name() << " Stoped" << std::endl;
	}
};