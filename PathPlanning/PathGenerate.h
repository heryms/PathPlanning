#ifndef _PATH_GENERATE_H
#define _PATH_GENERATE_H
#include "BaseType.h"
#include "Clothoid.h"
#include "CoordTransform.h"
class PathGenerate
{
public:
	PathGenerate()
	{
	}

	~PathGenerate()
	{
	}
	bool path_generate_grid(PosPoint startPt, PosPoint endPt);
	void path_generate_local(PosPoint startPt, PosPoint endPt);
private:

};


#endif