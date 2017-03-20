
#pragma once
#ifndef  _PATH_PLANNING_BASE_ANGLE_H
#define _PATH_PLANNING_BASE_ANGLE_H

#include <cmath>
#define ZRENGOUBIGITHUB
class RadAngle
{
private:
	double value;
public:
	RadAngle(){};
	~RadAngle(){};

	inline static double Normalize(double value){
#ifdef PI
#undef PI
#endif
#define PI 3.141592653589793
		int n = (int)(value / (2 * PI));
		value -= n*(2 * PI);
		if (value >= PI) {
			value -= 2 * PI;
		}
		if (value < -PI) {
			value += 2 * PI;
		}
		return value;
	}

	RadAngle(double v){
		value = Normalize(v);
	}

	operator double() const {
		return value;
	}

	RadAngle operator = (const double & v) {
		value = Normalize(v);
		return *this;
	}

	RadAngle operator + (const double & v) const {
		return RadAngle(Normalize(value + v));
	}

	RadAngle operator - (const double & v) {
		return RadAngle(Normalize(value - v));
	}

	RadAngle& operator +=(const double & v) {
		value = Normalize(value + v);
		return *this;
	}

	RadAngle& operator -=(const double & v) {
		value = Normalize(value - v);
		return *this;
	}

	bool operator ==(const double & v) const {
		return (value) == (Normalize(v));
	}
	bool operator !=(const double & v) const {
		return (value) != (Normalize(v));
	}
};

class DegAngle
{
private:
	double value;
public:
	DegAngle(){};

	~DegAngle(){};

	inline static double Normalize(double value){
		int n = (int)(value / (360));
		value -= n*(360);
		if (value >= 180) {
			value -= 360;
		}
		if (value < -180) {
			value += 360;
		}
		return value;
	}

	DegAngle(double v){
		value = Normalize(v);
	}

	operator double() const {
		return value;
	}

	DegAngle operator = (const double & v) {
		value = Normalize(v);
		return *this;
	}

	DegAngle operator + (const double & v) const {
		return DegAngle(Normalize(value + v));
	}

	DegAngle operator - (const double & v) {
		return DegAngle(Normalize(value - v));
	}

	DegAngle& operator +=(const double & v) {
		value = Normalize(value + v);
		return *this;
	}

	DegAngle& operator -=(const double & v) {
		value = Normalize(value - v);
		return *this;
	}

	bool operator ==(const double & v) const {
		return (value) == (Normalize(v));
	}
	bool operator !=(const double & v) const {
		return (value) != (Normalize(v));
	}
};
#endif //  _PATH_PLANNING_BASE_ANGLE_H