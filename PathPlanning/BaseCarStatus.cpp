#include "BaseCarStatus.h"


BaseCarStatus::BaseCarStatus()
{
}

BaseCarStatus::~BaseCarStatus()
{
}

PosPoint BaseCarStatus::GetPosition()
{
	return m_curPos;
}

double BaseCarStatus::GetSpeed()
{
	return m_speed;
}

double BaseCarStatus::GetSteerAngle()
{
	return m_steerAngle;
}

//double BaseCarStatus::GetWheelAngle()
//{
//	return m_wheelAngle;
//}

double BaseCarStatus::GetTargetSpeed()
{
	return m_targetSpeed;
}

double BaseCarStatus::GetTargetSteerAngle()
{
	return m_targetSteerAngle;
}

void BaseCarStatus::SendControl(double targetSpeed, double targetSteerAngle)
{
	m_targetSpeed = targetSpeed;
	m_targetSteerAngle = targetSteerAngle;
}

void BaseCarStatus::SetAcceleration(double acceleration)
{
	m_acceleration = acceleration;
}

void BaseCarStatus::SetSteerRotateRate(double steerRotateRate)
{
	m_steerRotateRate = steerRotateRate;
}

void BaseCarStatus::SetCurPosition(PosPoint position)
{
	m_curPos = position;
}

void BaseCarStatus::SetSpeed(double speed)
{
	m_speed = speed;
}

void BaseCarStatus::SetSteerAngle(double steerAngle)
{
	m_steerAngle = steerAngle;
}

void BaseCarStatus::SetTargetSpeed(double speed)
{
	m_targetSpeed = speed;
}

void BaseCarStatus::SetTargetSteerAngle(double steerAngle)
{
	m_targetSteerAngle = steerAngle;
}

void BaseCarStatus::Start()
{
}

void BaseCarStatus::Suspend()
{
}

void BaseCarStatus::Resume()
{
}

void BaseCarStatus::End()
{
}
