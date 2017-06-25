#include "PathGenerate.h"
#include <assert.h>
#include <math.h>
#include <fstream>
#include "lcmtype/PlanOutput.hpp"
#include "PathJoint.h"
#include <chrono>
using ckLcmType::PlanOutput;

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
	int delta_Grid_start = -5;
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
				pt.x = x;
				pt.y = y;
				fprintf(fp, "%lf %lf %lf\n", x, y, pt.angle);
			}
#ifdef LOG_CLOTHOID
			fclose(fp);
#endif // 
			m_sendPath.SendDraw(draw);
			track.SetPath(rdpt);
			send_succeed = true;
			break;

		}
	}
	if (!send_succeed)
	{
		std::cout << "no path" << std::endl;
		track.SetPath(std::vector<RoadPoint>());
		//TODO: step five send message about how to stop
		CarInfo info;
		info.speed = 0;
		info.state = E_STOP;
		info.steerAngle = 0;
		info.gear = D;
		CarControl::GetInstance().SendCommand(info);
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
bool  PathGenerate::path_generate_grid_obstacle(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids, std::vector<RoadPoint>& rdPt, int *y, int *x){
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
					*y = (int)rdPt[i].y;
					*x = (int)(rdPt[i].x + 0.5) + j;
					return false;
				}
			}
		}


	}
	return true;
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

void PathGenerate::createClothoidTable(){


	// step one define the start point and its directions
	PosPoint start_pt, endPt;
	start_pt.x = 75;
	start_pt.y = 200;
	start_pt.angle = PI / 2;

	clothoidMap.clear();
	clothoidMap.resize((GRID_END - GRID_START) *(ANGLE_END - ANGLE_START));
	// step two define the end point and create some directions
	target_Y = 250;
	target_X = 75;
	for (int i = GRID_START; i < GRID_END; i++)
	{

		endPt.y = target_Y;
		endPt.x = target_X - i;
		for (int angle = ANGLE_START; angle < ANGLE_END;angle++)
		{
			endPt.angle = angle / 180.0 *PI;
			std::vector<RoadPoint> rdPt;
			generateClothoidPoints(start_pt, endPt, rdPt);
			clothoidMap[(GRID_END - GRID_START)* (angle - ANGLE_START)+ (i - GRID_START)] = rdPt;
		}
		
	}


}
void PathGenerate::generateClothoidPoints(PosPoint startPt, PosPoint endPt, std::vector<RoadPoint>& rdPt){
	
	// create clothoid curve
	Clothoid path_clothoid(startPt.x, startPt.y, startPt.angle, endPt.x, endPt.y, endPt.angle);

	// get roadPoints
	int num_pt = 100;
	rdPt.clear();
	rdPt.resize(num_pt);// = new RoadPoint[num_pt];
	path_clothoid.PointsOnClothoid(rdPt, num_pt);
}
std::vector<RoadPoint> PathGenerate::getRdPtFromTable(int grid, int angle){

	int index = (GRID_END - GRID_START) * (angle - ANGLE_START)+ grid - GRID_START;
	assert(index >= 0 && index <= clothoidMap.size());
	return clothoidMap[index];
}
void PathGenerate::path_generate_using_bug()
{
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
	target_Y = 250;
	double target_Angle = DataCenter::GetInstance().GetRoadEdgePoint(target_Y, RIGHT).angle;
	// step three generate the path
	int delta_Grid_start = -5;
	int delta_Grid_end = 20;
	PosPoint startPt, endPt;
	startPt.x = 75;
	startPt.y = 200;
	startPt.angle = 90 / 180.0 * PI;
	std::vector<std::vector<RoadPoint>> success_root;
	for (int i = GRID_START; i < GRID_END; i++)
	{

		endPt.y = target_Y;
		endPt.x = target_X - i;
		for (int angle = ANGLE_START; angle < ANGLE_END; angle++)
		{
			
			std::vector<RoadPoint> rdPt;
			rdPt = getRdPtFromTable(i, angle);
			if (Topology::check_velogrid_rdPt_intersected(veloGrids, rdPt))
			{
				success_root.push_back(rdPt);
			}
			
		}
	}

	//store and compare the difference of the road
	if (pre_Root.size() == 0)
	{
		if (success_root.size() == 0)
		{
			std::cout << "no path" << std::endl;
			track.SetPath(std::vector<RoadPoint>());
			//TODO: step five send message about how to stop
			CarInfo info;
			info.speed = 0;
			info.state = E_STOP;
			info.steerAngle = 0;
			info.gear = D;
			CarControl::GetInstance().SendCommand(info);
			return;
		}
		else
		{
			// random choose first one
			// or you can give a direction 
			// then i can compute the cost
			// then choose a better one
			// random choose may cause a problem
			pre_Root = success_root[0];
		}
		
	}
	else
	{
		if (success_root.size() == 0 )
		{
			// choice using the pre the go 
			// or stop
			std::cout << "no path" << std::endl;
			track.SetPath(std::vector<RoadPoint>());
			//TODO: step five send message about how to stop
			CarInfo info;
			info.speed = 0;
			info.state = E_STOP;
			info.steerAngle = 0;
			info.gear = D;
			CarControl::GetInstance().SendCommand(info);
		}
		else{
			int simi_max = -1000;
			std::vector<RoadPoint> path=success_root[0];
			for (auto root : success_root)
			{
				double simi = similarity(pre_Root, root);
				if (simi > simi_max){
					simi_max = simi;
					path = root;
				}
			}
			// send the data
			ckLcmType::DecisionDraw_t draw;
			draw.num = path.size();
			draw.Path_x.reserve(draw.num);
			draw.Path_y.reserve(draw.num);
			for (RoadPoint& pt : path) {
				double x, y;
				CoordTransform::GridtoLocal(pt.x + X_START, pt.y - Y_START, x, y);
				draw.Path_x.push_back(x);
				draw.Path_y.push_back(y);
				//fprintf(fp, "%lf %lf %lf\n", x, y, pt.angle);
			}
			pre_Root = path;
			track.SetPath(path);
			m_sendPath.SendDraw(draw);
			
		}
		
	}

}
bool PathGenerate::path_generate_turning(VeloGrid_t& veloGrids){
	// define search region
	// lane width 3 so 6 grid
	int RegionWidth = 0;
	int RegionHeight = 0;
	int RegionStartX = 0;
	int RegionStartY = 0;

	PosPoint endPt, startPt;
	startPt.x = 75;
	startPt.y = 200;
	startPt.angle = PI / 2.;


	if (PreDirection == TURN_LEFT)
	{
		RegionStartX = 75 - 12;
		RegionStartY = 200 + 12;
		RegionWidth = 4;
		RegionHeight = 4;
		endPt.angle = PI;

	}
	else if (PreDirection == TUN_RIGHT)
	{
		RegionStartX = 75 + 6;
		RegionStartY = 200 + 12;
		RegionWidth = 4;
		RegionHeight = 4;
		endPt.angle = 0;
	}
	else
	{
		return false;
	}

	//
	


	for (int i = 0; i < RegionWidth;i++)
	{
		for (int j = 0; j < RegionHeight;j++)
		{
			endPt.x = RegionStartX + i;
			endPt.y = RegionStartY + j;
			std::vector<RoadPoint> rdpt;
			if (path_generate_grid_obstacle(startPt, endPt, veloGrids, rdpt)) {
				std::cout << "congratulations a successful root" << std::endl;

				// pass the data
			}

		}
	}
	
}
bool PathGenerate::path_generate_for_fun(){
	// step one receive data
	if (!DataCenter::GetInstance().HasVeloGrid()) {
		std::cout << "Warning::not velogrid message" << std::endl;
		return false;
	}
	if (!DataCenter::GetInstance().HasCurb()) {
		std::cout << "Warning::not curb message" << std::endl;
		return false;
	}
	VeloGrid_t veloGrids = DataCenter::GetInstance().GetLidarData();
	// step two get the target points and target direction
	target_X = 75;
	target_Y = 300;
	double target_Angle = DataCenter::GetInstance().GetRoadEdgePoint(target_Y, RIGHT).angle;

	//
	std::vector<PosPoint> obstacles;
	PosPoint startPt;
	PosPoint endPt;

	startPt.x = 75;
	startPt.y = 200;
	startPt.angle = 90 / 180.0 * PI;
	endPt.angle = startPt.angle;
	endPt.x = 75;
	endPt.y = 300;
	std::vector<PosPoint> root;
	bool result = path_generate_recursive(startPt, endPt, veloGrids,root, 0);
	if (!result)
	{
		std::cout << "no root "<<std::endl;
		return false;
	}
	if (posPtOnRoot.size() == 0)
	{
		return false;
	}
	else
	{
		ckLcmType::DecisionDraw_t draw;
		std::cout << "we have roads number" << posPtOnRoot.size() << std::endl;
		// how to choose one ?
		int total_num = 0.0;
		for (int i = 0; i < posPtOnRoot[0].size();i++)

		{
			endPt = posPtOnRoot[0][i];
			std::vector<RoadPoint> rdpt;
			if (path_generate_grid_obstacle(startPt, endPt, veloGrids, rdpt)) {
				//std::cout << "congratulations a successful root" << std::endl;

				
				
				for (RoadPoint& pt : rdpt) {
					double x, y;
					CoordTransform::GridtoLocal(pt.x + X_START, pt.y - Y_START, x, y);
					draw.Path_x.push_back(x);
					draw.Path_y.push_back(y);
					pt.x = x;
					pt.y = y;
					total_num += rdpt.size();

				}
			}
			startPt = endPt;
		}
		draw.num = total_num;
		//draw.Path_x.reserve(draw.num);
		//draw.Path_y.reserve(draw.num);
		m_sendPath.SendDraw(draw);
	}


	
}
bool PathGenerate::path_generate_recursive(PosPoint startPt, PosPoint endPt, VeloGrid_t veloGrids, std::vector<PosPoint> &root, int count){

	// first generate a path
	if (count>5)
	{
		return false;
	}
	std::vector<RoadPoint> rdPt;
	int obstacle_y = -1, obstacle_x = -1;

	// search Left
	for (int i = SAFE_REGION_START; i <= SAFE_REGION_END;i++)
	{
		endPt.x = endPt.x - i;
		if (path_generate_grid_obstacle(startPt, endPt, veloGrids, rdPt, &obstacle_y, &obstacle_x))
		{
			root.push_back(endPt);
			if (endPt.y != target_Y)
			{
				PosPoint startPt_tmp = endPt;
				endPt.x = target_X;
				endPt.y = target_Y;
				path_generate_recursive(startPt_tmp, endPt, veloGrids, root, count + 1);
				root.pop_back();
			}
			else
			{
				// reach the target point;
				posPtOnRoot.push_back(root);
			}
			return true;
		}
		else
		{
			endPt.y = obstacle_y;
			endPt.x = obstacle_x;
			root.push_back(endPt);
			path_generate_recursive(startPt, endPt, veloGrids, root, count+1);
			root.pop_back();
		}
	}

}
//bool PathGenerate
//void PathGenerate::motion_model(){
//
//	double theta, x, y, kappa, vt;
//	double deltaT = 0.1;
//	get_current_state();
//	double x_deltaT = x + vt * cos(theta) * deltaT;
//	double y_deltaT = y + vt * sin(theta) * deltaT;
//	double theta_deltaT = theta + vt*kappa*deltaT;
//
//}
double PathGenerate::dynamic_response(double k_next, double k, double v_next, double v, double t, 
	double &k_response, double & v_response){


	double kmax = 0.1900;
	double kmin = -kmax;
	double delta_k_max = 0.1021;
	double delta_k_min = -delta_k_max;
	double delta_v_max = 2.000;
	double delta_v_min = -6.000;

	double delta_k = (k_next - k) / t;
	delta_k = fmin(delta_k, delta_k_max);
	delta_k = fmax(delta_k, delta_k_min);

	k_response = k + delta_k * t;
	k_response = fmin(k_response, kmax);
	k_response = fmax(k_response, kmin);

	speed_logic_control(k_next, v_next, v, v_response);
	double delta_v = (v_response - v) / t;
	delta_v = fmin(delta_v, delta_v_max);
	delta_v = fmax(delta_v, delta_v_min);
	v_response = v + delta_v * t;

	return true;

}


bool PathGenerate::speed_logic_control(double k_next, double v_next, double v_pre, double &v_response){
	double v = v_next;
	double a = 0.1681;
	double b = -0.0049;
	double safefactor = 1.0;
	double vscl = 4.00;
	double k_max = 0.1900;
	double kv_max = 0.1485;
	double v_max = fmax(vscl, (k_next - a) / b);
	double k_max_scl = fmin(k_max, a + b * v);
	if (k_next > k_max_scl)
		v = abs(safefactor * v_max);
	v_response = v * v_pre / abs(v_pre);
	//value = [v_response];
	return true;
}


std::vector<RoadPoint> PathGenerate::BestPathSelector(const std::vector<std::vector<RoadPoint>> cadidatePath)
{

}

bool PathGenerate::short_time_planning(float qf, float qi, float theta, double sf,
	VeloGrid_t veloGrids, SXYSpline spline, std::vector<RoadPoint> & pts, PosPoint curPt,
	double& firstObstacle_x, double& firstObstacle_y, int& index, double& new_sf) {
	//std::vector<float> s(x_ref.size(), 0);
	//std::vector<float> delta_x(x_ref.size(), 0);
	//std::vector<float> delta_y(x_ref.size(), 0);
	//std::vector<float> length(x_ref.size(), 0);
	//std::vector<PointPt> pt;

	// need vector operator?
	/*for (int i = 1; i < x_ref.size();i++)
	{
		s[i] = sqrt((x_ref[i] - x_ref[i - 1])*(x_ref[i] - x_ref[i - 1])+
			(y_ref[i] - y_ref[i - 1])*(y_ref[i] - y_ref[i - 1])) + s[i - 1];
		delta_x[i] = x_ref[i] - x_ref[i - 1];
		delta_y[i] = y_ref[i] - y_ref[i - 1];
		length[i] = sqrt(delta_x[i] * delta_x[i] + delta_y[i] * delta_y[i]);

	}*/
	//??
	float c = tan(theta);
	
	float a = 2 * (qi - qf) / (pow(sf, 3)) + c / pow(sf, 2);
	float b = 3 * (qf - qi) / (pow(sf, 2)) - 2 * c / sf;

	/*bool firstObstacle = false;
	double firstObstacle_x = 0.0;
	double firstObstacle_y = 0.0;*/

	bool firstObstacle = true;

	for (int i = 0; i < 100;i++)
	{
		double delta_s = sf / 100 * i;
		float q = a * pow(delta_s, 3) + b * pow(delta_s, 2) + c * delta_s + qi;
		double x=0.0, y=0.0;
		spline.getXY(delta_s, x, y);
		double _delta_x_, _delta_y_;
		spline.getDeriveXY(delta_s, _delta_x_, _delta_y_);
		//if (i == 0)
		//	std::cout << "delta_X: " << _delta_x_ << " delta_Y: " << _delta_y_ << std::endl;
		double _length_ = sqrt(pow(_delta_x_, 2) + pow(_delta_y_, 2));
		Eigen::Matrix2Xd norm_vec(2, 1), pre(2, 1);
		norm_vec(0, 0) = _delta_x_ / _length_;
		norm_vec(1, 0) = _delta_y_ / _length_;
		pre(0, 0) = x;
		pre(1, 0) = y;
		Eigen::Matrix2Xd result = Topology::rotate(PI / 2, norm_vec)*q + pre;
		RoadPoint tmp;
		tmp.x = result(0, 0);
		tmp.y = result(1, 0);

		tmp.angle = Topology::toAngle(_delta_x_, _delta_y_);

		int Grid_x = tmp.x / Grid + 75;
		int Grid_y = tmp.y / Grid + 200;

		firstObstacle_x = 0.0;
		firstObstacle_y = 0.0;
		new_sf = 0.0;
		//collision detection
		for (int j = -CAR_WIDTH; j <= CAR_WIDTH; j++)
		{
			for (int jj = 0; jj < 2 * CAR_HEIGHT;jj++)
			{
				if ((int)(Grid_x + j) >= 0 && ((int)(Grid_x + j)) <= MAP_WIDTH - 1 && (int)(Grid_y + i) <= MAP_HEIGHT - 1)
				{
					int index = MAP_WIDTH*(int)(Grid_y + i)+(int)(Grid_x)+j;
					if (veloGrids.velo_grid[index]) {
						//record first obstacle
						if (firstObstacle)
						{
							firstObstacle_x = ((int)(Grid_x + j) - 75) * Grid;
							firstObstacle_y = ((int)(Grid_y + i + 2) - 200) * Grid;
							index = i;
							new_sf = delta_s;
						}
						return false;
					}
				}
			}
			
		}
		/*if (i != 0){
			tmp.x -= pts[0].x;
			tmp.y -= pts[0].y;
		}*/
		
		pts.push_back(tmp);
	}
	//pts[0].x = 0;
	//pts[0].y = 0;
	return true;
	/*for (int i = 1; i < x_ref.size();i++)
	{
		float delta_s = s[i];
		float q = a * pow(delta_s , 3) + b * pow(delta_s ,2) + c *delta_s + qi;
		Eigen::Matrix2Xd norm_vec(1,2), pre(1,2);
		norm_vec(0, 0) = delta_x[i] / length[i];
		norm_vec(0, 1) = delta_y[i] / length[i];
		pre(0, 0) = x_ref[i-1];
		pre(0, 1) = y_ref[i-1];
		Eigen::Matrix2Xd result = Topology::rotate(PI / 2, norm_vec)*q + pre;
		PointPt tmp;
		tmp.x = result(0, 0);
		tmp.y = result(0, 1);
		int Grid_x = tmp.x / Grid;
		int Grid_y = tmp.y / Grid;
		for (int j = -CAR_WIDTH; j <= CAR_WIDTH; j++)
		{
			if ((int)(Grid_x + j) >= 0 && ((int)(Grid_x + j)) <= MAP_WIDTH - 1)
			{
				int index = MAP_WIDTH*(int)(Grid_y) + (int)(Grid_x) + j;
				if (veloGrids.velo_grid[index]) {
					
					return false;
				}
			}
		}
		pt.push_back(tmp);

	}*/

	
}
void PathGenerate::short_time_planning(){
	// step one receive data
	if (!DataCenter::GetInstance().HasVeloGrid()) {
		std::cout << "Warning::not velogrid message" << std::endl;
		return ;
	}
	if (!DataCenter::GetInstance().HasCurb()) {
		std::cout << "Warning::not curb message" << std::endl;
		return ;
	}
	if (!DataCenter::GetInstance().HasRefTrajectory()) {
		std::cout << "Warning::not ref message" << std::endl;
		return;
	}
	VeloGrid_t veloGrids = DataCenter::GetInstance().GetLidarData();
	PosPoint curCar;
	curCar = DataCenter::GetInstance().GetCurPosition();
	//在相对于车辆的坐标系里规划
	curCar.x = .0;
	curCar.y = .0;
	//std::vector<RoadPoint> refTrajectory = DataCenter::GetInstance().GetReferenceTrajectory(curCar);
	double tmp_qi = 0.0;
	std::vector<RoadPoint> refTrajectory = DataCenter::GetInstance().GetRefTrajectory_Qi(tmp_qi);


	SXYSpline spline;
	spline.init(refTrajectory);

	// get data and process
	// float theta, double sf

	 //get car data
	//car.angle = PI / 2 - DataCenter::GetInstance().GetCurPosition().angle;
	//PosPoint tmp_Pt = DataCenter::GetInstance().GetCurOnTrajectory();
	//car.x = tmp_Pt.x;
	//car.y = tmp_Pt.y;

	//if (Root_On_Gaussian.size() != 0)
	//{
	//	std::vector<RoadPoint> pre_to_now = PathJoint::Gaussian_To_Decare(Root_On_Gaussian, car.angle, car.x, car.y);
	//}
	//qi here is not used
	double qi = 0;
	
	double theta = 0;
	DataCenter::GetInstance().Get_InitAngle_Qi(&spline,theta, qi);

	double sf = spline.S[spline.S.size() - 1];
	//std::cout << "Sf:" << sf << std::endl;
	std::vector<std::vector<RoadPoint>> _root_;
	std::vector<RoadPoint> rdpt;
	std::vector<float> road_qf;
	//生成候选路径
	double firstObstacle_x = 0.0;
	double firstObstacle_y = 0.0;
	int firstObsIndex;
	double new_sf = 0.0;
	for (float i = 0; i < 12; i+=0.2){
		float qf = i - 6.;
		std::vector<RoadPoint> pts;
		if(short_time_planning(qf, tmp_qi, theta, sf, veloGrids, spline, pts, curCar, firstObstacle_x, firstObstacle_y, firstObsIndex, new_sf))
		{
			_root_.push_back(pts);
			road_qf.push_back(qf);
		}
	}
	if (_root_.size() == 0)
	{
		std::cout << "no root to use " << std::endl;
		return;
	}
	//选择最优路径
	std::vector<RoadPoint> _best_root_;
	if (pre_Root.size()==0)
	{
		pre_Root = _root_[0];
		_best_root_ = _root_[0];
		int diff_max = 1000000;
		_best_root_ = _root_[0];
		int qf_index = 0;
		double qf_max = 100000;
		int index = 0;
		RoadPoint lastPt = *refTrajectory.rbegin();
		lastPt.x -= curCar.x;
		lastPt.y -= curCar.y;
		for (auto root : _root_){
			double distance = Topology::Distance2(root[root.size() - 1], lastPt);// refTrajectory[refTrajectory.size() - 1]);
			//distance = abs(root[root.size() - 1].x - refTrajectory[refTrajectory.size() - 1].x);
			if (distance < diff_max)
			{
				diff_max = distance;
				_best_root_ = root;
			}
			/*if (abs(road_qf[index])<qf_max)
			{
				qf_max = abs(road_qf[index]);
				qf_index = index;
			}
			index++;*/
		}
		//_best_root_ = _root_[qf_index];
	}
	else
	{
		int diff_max = 1000000;
		_best_root_ = _root_[0];
		int qf_index=0;
		double qf_max = 100000;
		int index = 0;
		RoadPoint lastPt = *refTrajectory.rbegin();
		lastPt.x -= curCar.x;
		lastPt.y -= curCar.y;
		for (auto root : _root_)
		{
			double _diff_ = similarity(pre_Root, root, sf);
			//_diff_ *= (abs(road_qf[index]) + 5) / 10;
			double distance = Topology::Distance2(root[root.size() - 1], lastPt);// refTrajectory[refTrajectory.size() - 1]);
			//distance = abs(root[root.size() - 1].x - refTrajectory[refTrajectory.size() - 1].x);
			if (distance < diff_max)
			{
				diff_max = distance;
				_best_root_ = root;
			}

			
		}
		pre_Root = _best_root_;
		/*_best_root_ = _root_[qf_index];
		std::cout << "qf max:" << qf_max << std::endl;*/
		Root_On_Gaussian = PathJoint::Decare_To_Gaussian(_best_root_, car.angle, car.x, car.y);
		// send the data
		//ckLcmType::DecisionDraw_t draw;
		
	}
	long long tp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	std::ofstream of("plan_" + std::to_string(tp) + ".txt");
	//_best_root_ = _root_[];
	ckLcmType::DecisionDraw_t draw;
	//draw.num = _best_root_.size();
	//draw.Path_x.reserve(draw.num);
	//draw.Path_y.reserve(draw.num);
	//for (RoadPoint& pt : _best_root_) {
	//	//double x, y;
	//	//CoordTransform::GridtoLocal(pt.x + X_START, pt.y - Y_START, x, y);
	//	draw.Path_x.push_back(pt.x);
	//	draw.Path_y.push_back(pt.y);
	//	//of << pt.x << "\t" << pt.y << "\n";
	//	//fprintf(fp, "%lf %lf %lf\n", x, y, pt.angle);
	//}
	track.SetPath(_best_root_);

	//draw all lines
		int total_num = 0;
	for (int i = 0; i < _root_.size(); i++)
	{
		for (int j = 0; j < _root_[i].size(); j++)
		{
			double x = _root_[i][j].x;
			double y = _root_[i][j].y;
			draw.Path_x.push_back(x);
			draw.Path_y.push_back(y);
			if (i == 0)
			{
				RoadPoint pt_for_track;
				pt_for_track.x = x;
				pt_for_track.y = y;
				pt_for_track.angle = 0;
				rdpt.push_back(pt_for_track);
			}
		}
		total_num += _root_[i].size();
	}
	//track.SetPath(rdpt);
	std::cout << "root number" << _root_.size() << std::endl;
	draw.num = total_num;
	
	std::ofstream ff("ref_" + std::to_string(tp) + ".txt");
	int total_refnum = refTrajectory.size();
	draw.refnum = total_refnum + _best_root_.size();
	draw.Refer_x.reserve(total_refnum);
	draw.Refer_y.reserve(total_refnum);
	for (int i = 0; i < total_refnum; i++)
	{
		draw.Refer_x.push_back(refTrajectory[i].x - curCar.x);
		draw.Refer_y.push_back(refTrajectory[i].y - curCar.y);
		//ff << refTrajectory[i].x << "\t" << refTrajectory[i].y << "\n";
	}
	for (RoadPoint& pt : _best_root_) {
		//double x, y;
		//CoordTransform::GridtoLocal(pt.x + X_START, pt.y - Y_START, x, y);
		draw.Refer_x.push_back(pt.x);
		draw.Refer_y.push_back(pt.y);
		//of << pt.x << "\t" << pt.y << "\n";
		//fprintf(fp, "%lf %lf %lf\n", x, y, pt.angle);
	}
	of.close();
	ff.close();
	m_sendPath.SendDraw(draw);

}
void PathGenerate::short_time_segment()
{
	//receive data from sensor
	if (!DataCenter::GetInstance().HasVeloGrid()) {
		std::cout << "Warning::not velogrid message" << std::endl;
		return;
	}
	if (!DataCenter::GetInstance().HasCurb()) {
		std::cout << "Warning::not curb message" << std::endl;
		return;
	}
	if (!DataCenter::GetInstance().HasRefTrajectory()) {
		std::cout << "Warning::not ref message" << std::endl;
		return;
	}
	VeloGrid_t veloGrids = DataCenter::GetInstance().GetLidarData();
	PosPoint curCar;
	curCar = DataCenter::GetInstance().GetCurPosition();
	//planning in the car coordinate
	curCar.x = .0;
	curCar.y = .0;

	double tmp_qi = 0.0;
	std::vector<RoadPoint> refTrajectory = DataCenter::GetInstance().GetRefTrajectory_Qi(tmp_qi);

	SXYSpline spline;
	spline.init(refTrajectory);

	//qi here is not used
	double qi = 0;

	double theta = 0;
	DataCenter::GetInstance().Get_InitAngle_Qi(&spline, theta, qi);

	double sf = spline.S[spline.S.size() - 1];

	std::vector<std::vector<RoadPoint>> cadidatePath;
	std::vector<RoadPoint> rdpt;
	std::vector<float> road_qf;

	//generate candidate path
	double firstObstacle_x = 0.0;
	double firstObstacle_y = 0.0;
	int firstObsIndex;
	double tmp_sf = 0.0;
	for (float i = 0; i < 12; i += 0.2) {
		float qf = i - 6.;
		std::vector<RoadPoint> pts;
		if (short_time_planning(qf, tmp_qi, theta, sf, veloGrids, spline, pts, curCar, firstObstacle_x, firstObstacle_y, firstObsIndex, tmp_sf))
		{
			cadidatePath.push_back(pts);
			road_qf.push_back(qf);
		}
	}

	//if we can not find a path, we replace the goal point with first obstacle(a little further)
	//and do the replanning work
	if (cadidatePath.size() == 0)
	{
		//std::cout << "no path found " << std::endl;
		//TODO:trim spline and replan
		for (float i = 0; i < 12; i += 0.2) {
			float qf = i - 6.0;
			std::vector<RoadPoint> pts;
			if (short_time_planning(qf, tmp_qi, theta, tmp_sf, veloGrids, spline, pts, curCar, firstObstacle_x, firstObstacle_y, firstObsIndex, tmp_sf))
			{
				cadidatePath.push_back(pts);
				road_qf.push_back(qf);
			}
			if (cadidatePath.empty())
			{
				std::cout << "No validate path found" << std::endl;
				return;
			}
		}
		
	}
	//select best path
	std::vector<RoadPoint> _best_root_;
	if (pre_Root.size() == 0)
	{
		pre_Root = cadidatePath[0];
		_best_root_ = cadidatePath[0];
		int diff_max = 1000000;
		_best_root_ = cadidatePath[0];
		int qf_index = 0;
		double qf_max = 100000;
		int index = 0;
		RoadPoint lastPt = *refTrajectory.rbegin();
		lastPt.x -= curCar.x;
		lastPt.y -= curCar.y;
		for (auto root : cadidatePath) {
			double distance = Topology::Distance2(root[root.size() - 1], lastPt);// refTrajectory[refTrajectory.size() - 1]);
																				 //distance = abs(root[root.size() - 1].x - refTrajectory[refTrajectory.size() - 1].x);
			if (distance < diff_max)
			{
				diff_max = distance;
				_best_root_ = root;
			}
		}
	}
	else
	{
		int diff_max = 1000000;
		_best_root_ = cadidatePath[0];
		int qf_index = 0;
		double qf_max = 100000;
		int index = 0;
		RoadPoint lastPt = *refTrajectory.rbegin();
		lastPt.x -= curCar.x;
		lastPt.y -= curCar.y;
		for (auto root : cadidatePath)
		{
			double _diff_ = similarity(pre_Root, root, sf);
			//_diff_ *= (abs(road_qf[index]) + 5) / 10;
			double distance = Topology::Distance2(root[root.size() - 1], lastPt);// refTrajectory[refTrajectory.size() - 1]);
																				 //distance = abs(root[root.size() - 1].x - refTrajectory[refTrajectory.size() - 1].x);
			if (distance < diff_max)
			{
				diff_max = distance;
				_best_root_ = root;
			}

		}
		pre_Root = _best_root_;
		Root_On_Gaussian = PathJoint::Decare_To_Gaussian(_best_root_, car.angle, car.x, car.y);
	}
	long long tp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	std::ofstream of("plan_" + std::to_string(tp) + ".txt");

	ckLcmType::DecisionDraw_t draw;
	track.SetPath(_best_root_);

	//draw all lines
	int total_num = 0;
	for (int i = 0; i < cadidatePath.size(); i++)
	{
		for (int j = 0; j < cadidatePath[i].size(); j++)
		{
			double x = cadidatePath[i][j].x;
			double y = cadidatePath[i][j].y;
			draw.Path_x.push_back(x);
			draw.Path_y.push_back(y);
			if (i == 0)
			{
				RoadPoint pt_for_track;
				pt_for_track.x = x;
				pt_for_track.y = y;
				pt_for_track.angle = 0;
				rdpt.push_back(pt_for_track);
			}
		}
		total_num += cadidatePath[i].size();
	}
	//track.SetPath(rdpt);
	std::cout << "root number" << cadidatePath.size() << std::endl;
	draw.num = total_num;

	std::ofstream ff("ref_" + std::to_string(tp) + ".txt");
	int total_refnum = refTrajectory.size();
	draw.refnum = total_refnum + _best_root_.size();
	draw.Refer_x.reserve(total_refnum);
	draw.Refer_y.reserve(total_refnum);
	for (int i = 0; i < total_refnum; i++)
	{
		draw.Refer_x.push_back(refTrajectory[i].x - curCar.x);
		draw.Refer_y.push_back(refTrajectory[i].y - curCar.y);
		//ff << refTrajectory[i].x << "\t" << refTrajectory[i].y << "\n";
	}
	for (RoadPoint& pt : _best_root_) {
		//double x, y;
		//CoordTransform::GridtoLocal(pt.x + X_START, pt.y - Y_START, x, y);
		draw.Refer_x.push_back(pt.x);
		draw.Refer_y.push_back(pt.y);
		//of << pt.x << "\t" << pt.y << "\n";
		//fprintf(fp, "%lf %lf %lf\n", x, y, pt.angle);
	}
	of.close();
	ff.close();
	m_sendPath.SendDraw(draw);
}
bool PathGenerate::cmu_planning(std::vector<double> k, double vt, double sf, 
	double theta, double x_start, double y_start, double delta_t){
	std::vector<double> x(k.size(), 0);
	std::vector<double> y(k.size(), 0);
	double k_response;
	double v_response;
	for (int i = 1; i < k.size();i++)
	{
		x[i] = x[i - 1] + vt * cos(theta)*delta_t;
		y[i] = y[i - 1] + vt * sin(theta)*delta_t;
		double k_tmp;
		if (i == 1)
			k_tmp = k[i - 1];
		else
			k_tmp = k_response;
		theta = theta + vt * k_tmp * delta_t;
		//s = sqrt((x(i + 1) - x(i)). ^ 2 + (y(i + 1) - y(i)). ^ 2) + s;
		double k_next = k[i];
		dynamic_response(k_next, k_tmp, vt, vt, delta_t, k_response, v_response);
		//k_response = value(1);
	}
	return true;
}