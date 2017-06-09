#ifndef ONLYTRACKING_BASE_CAR_STATUS_HH
#define ONLYTRACKING_BASE_CAR_STATUS_HH
#include "BaseType.h"

class BaseCarStatus
{
protected:
	double L = 2.67;
	double m_steerRatio = 17.3;
	PosPoint m_curPos;
	double m_speed;//km/h
	double m_steerAngle;//deg
	//double m_wheelAngle;//deg
	double m_targetSpeed;//km/h
	double m_targetSteerAngle;//deg
	double m_acceleration;//m/s2
	double m_steerRotateRate;//deg/s
public:
	BaseCarStatus();
	virtual ~BaseCarStatus();
	/*@return position orientation by x in rad*/
	virtual PosPoint GetPosition();
	/*@return speed in km/h*/
	virtual double GetSpeed();
	/*@return steer angle in deg*/
	virtual double GetSteerAngle();
	///*@return wheel angle in deg*/
	//virtual double GetWheelAngle();
	/*@return target speed in km/h*/
	virtual double GetTargetSpeed();
	/*@return target steer angle in deg*/
	virtual double GetTargetSteerAngle();
	/**
	* @param targetspeed in km/h
	* @param targetsteerangle in deg
	**/
	virtual void SendControl(double targetSpeed, double targetSteerAngle);
	/*@param acceleration in m/s2*/
	virtual void SetAcceleration(double acceleration);
	/*@param steer roate rate in deg/s*/
	virtual void SetSteerRotateRate(double steerRotateRate);
	/*@param position orientation by x in rad*/
	virtual void SetCurPosition(PosPoint position);
	/*@param speed in km/h*/
	virtual void SetSpeed(double speed);
	/*@param steer angle in deg*/
	virtual void SetSteerAngle(double steerAngle);
	/*@param target speed in km/h*/
	virtual void SetTargetSpeed(double speed);
	/*@param target steer angle in deg*/
	virtual void SetTargetSteerAngle(double steerAngle);

	virtual void Start();
	virtual void Suspend();
	virtual void Resume();
	virtual void End();

	inline double GetL() {
		return L;
	}
	inline double GetSteerRatio() {
		return m_steerRatio;
	}
};
#endif

