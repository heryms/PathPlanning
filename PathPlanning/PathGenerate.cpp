#include "PathGenerate.h"
#define LOG_CLOTHOID
bool PathGenerate::path_generate_grid(PosPoint startPt, PosPoint endPt, ckLcmType::VeloGrid_t& veloGrids, std::vector<RoadPoint>& rdPt) {

	//VeloGrid_t veloGrids=DataCenter::GetInstance().GetLidarData();

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
	rdPt.resize(num_pt);// = new RoadPoint[num_pt];
	path_clothoid.PointsOnClothoid(rdPt, num_pt);

	//

	for (int i = 0; i < num_pt; i++)
	{
		// may be a bug can be saver
		/*for (int j = -4; j <= 4;j++)
		{
			if (((int)rdPt[i].x + j) >= 0 && ((int)rdPt[i].x + j) <= MAP_WIDTH -1)
				grid_map[MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x + j] = 1;
		}*/
		if (((int)rdPt[i].x + CAR_WIDTH) >= 0 && ((int)rdPt[i].x + CAR_WIDTH) <= MAP_WIDTH - 1)
		{
			grid_map_end[(int)rdPt[i].y] =
				(int)rdPt[i].x + CAR_WIDTH > grid_map_end[(int)rdPt[i].y]
				?
				(int)rdPt[i].x + CAR_WIDTH : grid_map_end[(int)rdPt[i].y];

		}
		else {
			grid_map_end[(int)rdPt[i].y] = MAP_WIDTH - 1;
		}
		if (((int)rdPt[i].x - CAR_WIDTH) >= 0 && ((int)rdPt[i].x - CAR_WIDTH) <= MAP_WIDTH - 1)
		{
			grid_map_start[(int)rdPt[i].y] =
				(int)rdPt[i].x - CAR_WIDTH < grid_map_start[(int)rdPt[i].y]
				?
				(int)rdPt[i].x - CAR_WIDTH : grid_map_start[(int)rdPt[i].y];
		}
		else {
			grid_map_start[(int)rdPt[i].y] = 0;
		}

	}

	//
#ifdef TEST
	for (int i = 0; i < num_pt; i++) {

		for (int j = -CAR_WIDTH; j <= CAR_WIDTH; j++)
		{
			if (((int)rdPt[i].x + j) >= 0 && ((int)rdPt[i].x + j) <= MAP_WIDTH - 1)
			{
				index = [MAP_WIDTH*(int)rdPt[i].y + (int)rdPt[i].x + j];
				if (veloGrids.velo_grid[index]) {
					return false
				}
			}

		}
	}
#endif // !TEST

	//int obstacle_size;
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

bool PathGenerate::path_generate_local(PosPoint startPt, PosPoint endPt) {
	//// create clothoid curve
	//Clothoid path_clothoid(startPt.x, startPt.y, startPt.angle, endPt.x, endPt.y, endPt.angle);

	//// get roadPoints
	//int num_pt = 100;
	//RoadPoint *rdPt = new RoadPoint[num_pt];
	//RoadPoint *gridPt = new RoadPoint[num_pt];
	//path_clothoid.PointsOnClothoid(rdPt, num_pt);
	//// change to grid
	//for (auto i = 0; i < num_pt; i++)
	//{
	//	int x = 0;
	//	int y = 0;
	//	PosPoint pt;
	//	pt.x = rdPt[i].x;
	//	pt.y = rdPt[i].y;
	//	pt.angle = rdPt[i].angle;
	//	CoordTransform::LocaltoGrid(pt, x, y);
	//	gridPt[i].x = x;
	//	gridPt[i].y = y;
	//	gridPt[i].angle = rdPt[i].angle;
	//	gridPt[i].changeangle = rdPt[i].changeangle;
	//}
	//// define whether intersected or not

	//// step one get rightest point
	//int index = getRightestPoints(rdPt, num_pt);
	//// step two consider the car width
	//rdPt[index].x += 0.9;   

	//// TODO get the a,b,c
	//laserCurbs::pointXYZI coeff = DataCenter::GetInstance().GetRoadEdgeCoefficient(RIGHT);
	//double a, b, c;
	//a = coeff.x;
	//b = coeff.y;
	//c = coeff.z;
	//double distance = abs(a*rdPt[index].x + b*rdPt[index].y + c) / sqrtf(a*a + b*b);
	//if (distance<0.3)
	//{
	//	return false;
	//}
	//else
	//{
	return true;
	//}


}

void PathGenerate::path_generate() {

	// step one receive data
	if (!DataCenter::GetInstance().HasVeloGrid()) {
		std::cout << "Warning::not velogrid message" << std::endl;
		return;
	}
	if (!DataCenter::GetInstance().HasCurb()) {
		std::cout << "Warning::not curb message" << std::endl;
		return;
	}
	VeloGrid_t veloGrids = DataCenter::GetInstance().GetLidarData();
	// step two get the target points and target direction
	target_X = 75;
	target_Y = 400;
	double target_Angle = DataCenter::GetInstance().GetRoadEdgePoint(target_Y, RIGHT).angle;
	// step three generate the path
	int delta_Grid_start = -20;
	int delta_Grid_end = 20;
	PosPoint startPt, endPt;
	startPt.x = 75;
	startPt.y = 200;
	startPt.angle = 90 / 180.0 * PI;
	bool send_succeed = false;
	for (int i = delta_Grid_start; i < delta_Grid_end; i++)
	{
		endPt.y = target_Y;
		endPt.x = target_X - i;
		endPt.angle = target_Angle;
		//TODO set angle
		std::vector<RoadPoint> rdpt;
		if (path_generate_grid_obstacle(startPt, endPt, veloGrids, rdpt)) {
			std::cout << "congratulations a successful root" << std::endl;
			// TODO: step four send the data

#ifdef LOG_CLOTHOID

			FILE *fp = fopen("clothoid.txt", "a+");
#endif // LOG_CLOTHOID
			ckLcmType::DecisionDraw_t draw;
			draw.num = rdpt.size();
			draw.Path_x.reserve(draw.num);
			draw.Path_y.reserve(draw.num);
			for (RoadPoint& pt : rdpt) {
				double x, y;
				CoordTransform::GridtoLocal(pt.x + X_START, pt.y - Y_START, x, y);
				draw.Path_x.push_back(x);
				draw.Path_y.push_back(y);
				fprintf(fp, "%lf %lf %lf\n", x, y, pt.angle);
			}
#ifdef LOG_CLOTHOID
			fclose(fp);
#endif // 
			m_sendPath.SendDraw(draw);
			send_succeed = true;
			break;

		}
	}
	if (!send_succeed)
	{
		std::cout << "no path" << std::endl;
		//TODO: step five send message about how to stop
		CarInfo info;
		info.speed = 0;
		info.state = E_STOP;
		info.steerAngle = 0;
		info.gear = D;
		m_sendControl.SendCommand(info);
	}

}

double PathGenerate::getTargetDirection() {
	return 0.0;
}

int PathGenerate::getRightestPoints(RoadPoint *rdPt, int numPt) {

	int maxVal_x = rdPt[0].x;
	int index = 0;
	for (int i = 0; i < numPt; i++)
	{
		if (rdPt[i].x > maxVal_x)
		{
			maxVal_x = rdPt[i].x;
			index = i;
		}
	}
	return index;
}

bool PathGenerate::path_generate_grid_obstacle(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids, std::vector<RoadPoint>& rdPt)
{
	// 
	int x_start = startPt.x;
	int y_start = startPt.y;
	int x_end = endPt.x;
	int y_end = endPt.y;
	// TRANSFORM TO GRID COORDINATE
//	CoordTransform::LocaltoGrid(startPt, x_start, y_start);
//	CoordTransform::LocaltoGrid(endPt, x_end, y_end);

	// create clothoid curve
	Clothoid path_clothoid(x_start, y_start, startPt.angle, x_end, y_end, endPt.angle);

	// get roadPoints
	int num_pt = 100;
	//rdPt.clear();
	rdPt.resize(num_pt);// = new RoadPoint[num_pt];
	path_clothoid.PointsOnClothoid(rdPt, num_pt);
	for (int i = 0; i < num_pt; i++)
	{
		// may be a bug can be saver
		for (int j = -CAR_WIDTH; j <= CAR_WIDTH; j++)
		{
			if ((int)(rdPt[i].x + 0.5 + j) >= 0 && ((int)(rdPt[i].x + 0.5 + j)) <= MAP_WIDTH - 1)
			{
				int index = MAP_WIDTH*(int)(rdPt[i].y + 0.5) + (int)(rdPt[i].x + 0.5) + j;
				if (veloGrids.velo_grid[index]) {
					return false;
				}
			}
		}


	}


	return true;
}