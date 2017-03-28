#ifndef _PATH_GENERATE_MATH_FILTER_TOOL_
#define _PATH_GENERATE_MATH_FILTER_TOOL_
#include <queue>
#include <numeric>
#include <cmath>
#include "Variables.h"

double getStd(std::vector<double> angle);
double getMean(std::vector<double> angle);
std::vector<double> getNormalDistribution(int sigma, int mean, int num);
#endif