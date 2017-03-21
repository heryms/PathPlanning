#include "PathGenerate.h"
#include "DataCenter.h"
bool PathGenerate::path_generate_grid(PosPoint startPt, PosPoint endPt){

	VeloGrid_t veloGrids=DataCenter::GetInstance().GetLidarData();

	// 
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
	delete[] grid_map_start;
	delete[] grid_map_end;

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
	// change to grid
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
	// define whether intersected or not

	// step one get rightest point
	int index = getRightestPoints(rdPt, num_pt);
	// step two consider the car width
	rdPt[index].x += 0.9;   

	// TODO get the a,b,c
	double a, b, c;
	double distance = abs(a*rdPt[index].x + b*rdPt[index].y + c) / sqrtf(a*a + b*b);
	if (distance<0.3)
	{
		return false;
	}
	else
	{
		return true;
	}


}
void PathGenerate::path_generate(){

	// step one receive data

	// step two get the target points and target direction

	// step three generate the path
	int delta_Grid_start = 4;
	int delta_Grid_end = 8;
	PosPoint startPt, endPt;
	startPt.x = 200;
	startPt.y = 75;
	startPt.angle = 90 / 180.0 * PI;
	bool send_succeed=false;
	for (int i = delta_Grid_start; i < delta_Grid_end;i++)
	{
		endPt.y = target_Y;
		endPt.x = target_X - i;
		//TODO set angle
		if (path_generate_grid(startPt, endPt)){
			std::cout << "congratulations a successful root" << std::endl;
			// TODO: step four send the data 
			send_succeed = true;
			break;

		}
	}
	if (!send_succeed)
	{
		//TODO: step five send message about how to stop

	}

}
double PathGenerate::getTargetDirection(){

}
int PathGenerate::getRightestPoints(RoadPoint *rdPt, int numPt){

	int maxVal_x = rdPt[0].x;
	int index = 0;
	for (int i = 0; i < numPt;i++)
	{
		if (rdPt[i].x > maxVal_x)
		{
			maxVal_x = rdPt[i].x;
			index = i;
		}
	}
	return index;
}