#include "CollisionDetection.h"

int CollisionDetection::collisionCheck(VeloGrid_t & grids, std::vector<RoadPoint>& localPath, bool hasAngle)
{
	double gridSize = 0.2;
	double carWidth = 2.4;
	double carLength = 3.7;

	double rear2Back = 0.8;//后轮轴到车屁股的长度

	for (int i = 0; i < localPath.size(); i++)
	{
		RoadPoint pt = localPath[i];
		RoadPoint co;
		//if the any point on path collides with the obstacle, return the point index. 
		if (CollisionDetection::detectObstacle(pt, grids, co))
			return i;
	}
	//default return -1, means the path does not collide with obstacles
	return -1;
}

bool CollisionDetection::detectObstacle(RoadPoint cur, VeloGrid_t & grids, RoadPoint & collisionpoint)
{
	Car myCar;
	myCar.Position = cur;
	myCar.length = 4.5;
	myCar.width = 2.8;
	myCar.RtoT = 1.5;


	//RoadPoint curPo = DataCenter::GetInstance().GetCurPosition();
	//double dx = myCar.Position.x;//- curPo.x;
	//double dy = myCar.Position.y;//- curPo.y;
	//double angle11 = Topology::AngleNormalnize1(PI / 2.0 - curPo.angle);
	//double cx = dx*cos(angle11) - dy*sin(angle11);
	//double cy = dx*sin(angle11) + dy*cos(angle11);

	//double angle = Topology::AngleNormalnize1(myCar.Position.angle + angle11);

	////double angle=myCar.Position.angle-g_CurrentLocation.angle;

	//myCar.Position.x = cx;
	//myCar.Position.y = cy;
	//myCar.Position.angle = angle;

	myCar.rearx = myCar.Position.x;
	myCar.reary = myCar.Position.y;
	myCar.phi = myCar.Position.angle;

	RoadPoint LeftRear;
	LeftRear.x = myCar.rearx - myCar.RtoT*cos(myCar.phi) - 0.5*myCar.width*sin(myCar.phi);
	LeftRear.y = myCar.reary - myCar.RtoT*sin(myCar.phi) + 0.5*myCar.width*cos(myCar.phi);

	//unsigned int LRx = LeftRear.x / 0.2 + GRID_WidthNum / 2;
	//unsigned int LRy = LeftRear.y / 0.2;

	double grid_widthX = 0.2;
	double grid_widthY = 0.2;
	//转化车所占的所有点到格网，并对其做检查
	for (double i = 0; i<myCar.width; i += grid_widthX)
		for (double j = 0; j<myCar.length; j += grid_widthY)
		{

			RoadPoint point;
			int GRID_WidthNum = 150;
			point.x = i*sin(myCar.phi) + j*cos(myCar.phi) + LeftRear.x;
			point.y = -i*cos(myCar.phi) + j*sin(myCar.phi) + LeftRear.y;
			int grid_X = point.x / grid_widthX + GRID_WidthNum / 2;
			int grid_Y = point.y / grid_widthY + 200;

			if (grid_X <= 0)
			{
				return false;
			}
			if (grid_Y <= 0)
			{
				return false;
			}
			if (grid_X >= 150)
			{
				return false;
			}
			if (grid_Y >= 400)
			{
				return false;
			}
			int aaa = grids.velo_grid[grid_Y*GRID_WidthNum + grid_X];
			if (aaa>0)
			{
				collisionpoint.x = point.x;
				collisionpoint.y = point.y;
				return true;//说明有障碍物
			}
		}

	return false;
}
