#include "PathGenerate.h"
#include "DataCenter.h"
bool PathGenerate::path_generate_grid(PosPoint startPt, PosPoint endPt){

	VeloGrid_t veloGrids=DataCenter::GetInstance().GetLidarData();

	// 
	int * grid_map = new int[MAP_HEIGHT*MAP_WIDTH];
	int * grid_map_start = new int[MAP_HEIGHT];
	int * grid_map_end = new int[MAP_HEIGHT];
	int x_start = 0;
	int y_start = 0;
	int x_end = 0;
	int y_end = 0;
	// TRANSFORM TO GRID COORDINATE
	CoordTransform::LocaltoGrid(startPt, x_start, y_start);
	CoordTransform::LocaltoGrid(endPt, x_end, y_end);

	// create clothoid curve
	Clothoid path_clothoid(x_start, y_start, startPt.angle, x_end, y_end, endPt.angle);

	// get roadPoints
	int num_pt = 100;
	RoadPoint *rdPt = new RoadPoint[num_pt];
	path_clothoid.PointsOnClothoid(rdPt, num_pt);

	//
	for (int i = 0; i < num_pt;i++)
	{
		// may be a bug can be saver
		/*for (int j = -4; j <= 4;j++)
		{
			if (((int)rdPt[i].x + j) >= 0 && ((int)rdPt[i].x + j) <= MAP_WIDTH -1)
				grid_map[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x + j] = 1;
		}*/
		if (((int)rdPt[i].x + 9) >= 0 && ((int)rdPt[i].x + 9) <= MAP_WIDTH - 1)
		{
			grid_map_end[(int)rdPt[i].y] =
				(int)rdPt[i].x + 9 > grid_map_end[(int)rdPt[i].y] 
				?
				(int)rdPt[i].x + 9 : grid_map_end[(int)rdPt[i].y];
			
		}
		else{
			grid_map_end[(int)rdPt[i].y] = MAP_WIDTH - 1;
		}
		if (((int)rdPt[i].x - 9) >= 0 && ((int)rdPt[i].x - 9) <= MAP_WIDTH - 1)
		{
			grid_map_start[(int)rdPt[i].y] = 
				(int)rdPt[i].x - 9 < grid_map_start[(int)rdPt[i].y]
				?
				(int)rdPt[i].x - 9 : grid_map_start[(int)rdPt[i].y];
		}
		else{
			grid_map_start[(int)rdPt[i].y] = 0;
		}
		
	}

	//
	
	int obstacle_size;
	for (size_t i = 0; i < veloGrids.height; i++)
	{
		for (size_t j = 0; j < veloGrids.width; j++)
		{
			int index = i*veloGrids.width + j;
			if (veloGrids.velo_grid[i])
			{
				bool flag = j >= grid_map_start[j] && grid_map_end[j] <= j;
				if (flag)
				{
					return false;
				}
			}
		}
	}

	return true;
	

}
bool PathGenerate::path_generate_local(PosPoint startPt, PosPoint endPt){
	// create clothoid curve
	Clothoid path_clothoid(startPt.x, startPt.y, startPt.angle, endPt.x, endPt.y, endPt.angle);

	// get roadPoints
	int num_pt = 100;
	RoadPoint *rdPt = new RoadPoint[num_pt];
	RoadPoint *gridPt = new RoadPoint[num_pt];
	path_clothoid.PointsOnClothoid(rdPt, num_pt);
	//
	for (auto i = 0; i < num_pt; i++)
	{
		int x = 0;
		int y = 0;
		PosPoint pt;
		pt.x = rdPt[i].x;
		pt.y = rdPt[i].y;
		pt.angle = rdPt[i].angle;
		CoordTransform::LocaltoGrid(pt, x, y);
		gridPt[i].x = x;
		gridPt[i].y = y;
		gridPt[i].angle = rdPt[i].angle;
		gridPt[i].changeangle = rdPt[i].changeangle;
	}
}