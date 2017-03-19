#include "BaseAngle.h"

RadAngle::RadAngle()
	:value(0){

}

RadAngle::RadAngle(double v)
	:value(Normalize(v)){

}

RadAngle::~RadAngle(){

}

RadAngle::operator double() const {
	return value;
}

RadAngle RadAngle::operator = (const double & v) {
	value = Normalize(v);
	return *this;
}

RadAngle RadAngle::operator + (const double & v) const {
	return RadAngle(Normalize(value + v));
}

RadAngle RadAngle::operator - (const double & v) const{
	return RadAngle(Normalize(value - v));
}

RadAngle& RadAngle::operator +=(const double & v) {
	value = Normalize(value + v);
	return *this;
}

RadAngle& RadAngle::operator -=(const double & v) {
	value = Normalize(value - v);
	return *this;
}

bool RadAngle::operator ==(const double & v) const {
	return (value) == (Normalize(v));
}
bool RadAngle::operator !=(const double & v) const {
	return (value) != (Normalize(v));
}

DegAngle::DegAngle()
	:value(0){

}




DegAngle::DegAngle(double v)
	: value(Normalize(v)){

}

DegAngle::~DegAngle(){

}

DegAngle::operator double() const {
	return value;
}

DegAngle DegAngle::operator = (const double & v) {
	value = Normalize(v);
	return *this;
}

DegAngle DegAngle::operator + (const double & v) const {
	return DegAngle(Normalize(value + v));
}

DegAngle DegAngle::operator - (const double & v) const{
	return DegAngle(Normalize(value - v));
}

DegAngle& DegAngle::operator +=(const double & v) {
	value = Normalize(value + v);
	return *this;
}

DegAngle& DegAngle::operator -=(const double & v) {
	value = Normalize(value - v);
	return *this;
}

bool DegAngle::operator ==(const double & v) const {
	return (value) == (Normalize(v));
}
bool DegAngle::operator !=(const double & v) const {
	return (value) != (Normalize(v));
}