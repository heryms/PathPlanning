#include <iostream>
#include "LcmType_Handler.h"

using std::chrono::steady_clock;

template<typename T>
class LcmRecvHelper : public LcmHandler<T>{
private:
	steady_clock::time_point startTime;
	steady_clock::time_point endTime;
	bool hasMsg;
public:
	LcmRecvHelper() :LcmHandler<T>(), hasMsg(false){

	}

	bool HasLcmMessage(){
		return hasMsg;
	}

	bool RecvBody(){
		if (grabTimeout(20) > 0){
			startTime = steady_clock::now();
		}
		endTime = steady_clock::now();
		std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		if (time.count() > 500){
			hasMsg = false;
			std::cout << "Warning::May not " << typeid(T).name() << "!" << std::endl;
		}
		else{
			hasMsg = true;
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
		//__super::uninitialLcm();
		hasMsg = false;
		std::cout << typeid(T).name() << " Stoped" << std::endl;
	}
};