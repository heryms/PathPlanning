#pragma once

#define infcon 10000000
#define eps1 2.2204e-016

class ClothoidBuilder
{
public:
	ClothoidBuilder();
	~ClothoidBuilder();

protected:
	void FresnelCS(double *FresnelC, double *FresnelS, double y);
	


private:
	double _k;//斜率
	double _dk;//斜率变化率
	double _L;//Clothoid曲线长度
};

