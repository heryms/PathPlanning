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
std::vector<double> getNormalDistribution(int sigma, int mean, int num){
	std::vector<double> guassian_prob;
	guassian_prob.resize(num);
	double s = 1.0 / (sigma*sqrtf(2 * PI));
	for (int i = 0; i < num; i++)
	{
		guassian_prob[i] = s*exp(-(i - mean)*(i - mean) / (2 * sigma));
	}
	return guassian_prob;
}