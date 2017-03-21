#include "PathGenerate.h"
void path_generate_grid(PosPoint startPt, PosPoint endPt){
	
	// 
	int * grid_map = new int[MAP_HEIGHT*MAP_WIDTH];
	int * grid_map_start = new int[MAP_HEIGHT*MAP_WIDTH];
	int * grid_map_end = new int[MAP_HEIGHT*MAP_WIDTH];
	int x_start = 0;
	int y_start = 0;
	int x_end = 0;
	int y_end = 0;
	// TRANSFORM TO GRID COORDINATE
	CoordTransform::LocaltoGrid(startPt, x_start, y_start);
	CoordTransform::LocaltoGrid(endPt, x_end, y_end);

	// create clothoid curve
	Clothoid path_clothoid(x_start, y_start, startPt.Angle, x_end, y_end, endPt.Angle);

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
			grid_map_end[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x + 9] = (int)rdPt[i].x + 9 > grid_map_end[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x + 9] ?
				(int)rdPt[i].x + 9 : (int)rdPt[i].x + 9;
			
		}
		else{
			grid_map_end[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x + 9] = MAP_WIDTH - 1;
		}
		if (((int)rdPt[i].x - 9) >= 0 && ((int)rdPt[i].x - 9) <= MAP_WIDTH - 1)
		{
			grid_map_start[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x - 9] = (int)rdPt[i].x - 9 < grid_map_start[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x + 9] ?
				(int)rdPt[i].x - 9 : (int)rdPt[i].x - 9;
		}
		else{
			grid_map_start[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x - 9] = 0;
		}
		
	}

	//
	int * new_obstacle = new int[100];
	int obstacle_size;
	for (int i = 0; i < obstacle_size;i++)
	{
		int x, y;
		int start = grid_map_start[MAP_WIDTH * y + x];
		int end = grid_map_end[MAP_WIDTH*y + x];
		if (x >= start && x <= end)
			return ;

	}





}