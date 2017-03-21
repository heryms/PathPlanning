#ifndef _TEST_MODULE_
#define _TEST_MODULE_
//#include "CoordTransform.h"
#include "GaussProjection.h"
#include "BaseType.h"
#include "Clothoid.h"
#include "Variables.h"
void test_To_Guassian(
	double grad, double minute, double second,
	double grad1, double minute1, double second1);
void test_To_Guassian(
	double lon, double lat
	);
void test_Clothoid();

#endif
