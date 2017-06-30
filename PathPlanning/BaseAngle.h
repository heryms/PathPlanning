#pragma once
#ifndef _PATH_PLANNING_BASE_ANGLE_H
#define _PATH_PLANNING_BASE_ANGLE_H

class RadAngle
{
private:
	double value;
public:
	RadAngle();

	RadAngle(double v);

	~RadAngle();

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

	operator double() const;

	RadAngle operator = (const double & v);

	RadAngle operator + (const double & v) const;

	RadAngle operator - (const double & v) const;

	RadAngle& operator +=(const double & v);

	RadAngle& operator -=(const double & v);

	bool operator ==(const double & v) const;

	bool operator !=(const double & v) const;

	//ÄæÊ±Õë
	bool belong(RadAngle min, RadAngle max);
};

class DegAngle
{
private:
	double value;
public:
	DegAngle();

	DegAngle(double v);

	~DegAngle();

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


	operator double() const;

	DegAngle operator = (const double & v);

	DegAngle operator + (const double & v) const;

	DegAngle operator - (const double & v) const;

	DegAngle& operator +=(const double & v);

	DegAngle& operator -=(const double & v);

	bool operator ==(const double & v) const;

	bool operator !=(const double & v) const;
};
#endif //  _PATH_PLANNING_BASE_ANGLE_H