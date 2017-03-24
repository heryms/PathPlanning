#include "filterTool.h"

double getStd(std::vector<double> angle){
	if (angle.size() == 0)return 0;

	double mean = getMean(angle);
	double accum = 0.0;
	for (auto value:angle)
	{
		accum += (value - mean) *(value - mean);
	}
	return accum / angle.size();
}
double getMean(std::vector<double> angle){

	if (angle.size() == 0)
	{
		return 0;
	}
	return std::accumulate(angle.begin(), angle.end(), 0) / angle.size();
}