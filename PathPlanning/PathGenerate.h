#ifndef _PATH_GENERATE_H
#define _PATH_GENERATE_H
#include "BaseType.h"
#include "Clothoid.h"
#include "CoordTransform.h"
#include "DataCenter.h"
#include "Topology.h"
#include "CostFunc.h"
#include <vector>
#include <queue>
#include "PathDraw.h"
#include "CarControl.h"
#include "TrackHelper.h"
#include "PathGenerateTool.h"


#define VELOCITY_ENCORAGE_FACTOR 1.2

class PathGenerate
{
private:
	TrackHelper track;
	PathDraw m_sendPath;
public:
	PathGenerate()
	{
		createClothoidTable();
		track.Start();

		m_isSegmentMode = false;
		m_testVelocityFactor = 1.2;
		updatetra = true;
	}

	~PathGenerate()
	{
		track.End();
	}
	void createClothoidTable();
	void generateClothoidPoints(PosPoint startPt, PosPoint endPt, std::vector<RoadPoint>& genPoints);
	bool path_generate_grid(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids,std::vector<RoadPoint>& genPoints);
	bool path_generate_local(PosPoint startPt, PosPoint endPt);
	bool path_generate_grid_obstacle(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids, std::vector<RoadPoint>& genPoints);
	bool path_generate_grid_obstacle(PosPoint startPt, PosPoint endPt, VeloGrid_t& veloGrids, std::vector<RoadPoint>& genPoints, int *y, int *x);
	void path_generate();
	void path_generate_using_bug();
	std::vector<RoadPoint> getRdPtFromTable(int grid, int angle);
	int getRightestPoints(RoadPoint *rdPt, int num_Pt);
	double getTargetDirection();
	bool path_generate_turning(VeloGrid_t& veloGrids);
	bool path_generate_for_fun();
	bool path_generate_recursive(PosPoint startPt, PosPoint endPt, VeloGrid_t veloGrids, 
		std::vector<PosPoint> &root, int count);
	int short_time_planning(float qf, float qi, float theta, double sf, 
		VeloGrid_t veloGrids, SXYSpline spline, std::vector<RoadPoint> & pts,PosPoint curPt, double& firstObstacle_x, double& firstObstacle_y, int& index,
		double& new_sf);
	std::vector<RoadPoint> trajectory_build(float qf,float qi,float theta,double sf,SXYSpline& spline);
	void short_time_planning();
	void short_time_segment();
	bool cmu_planning(std::vector<double> k, double vt, double sf, 
		double theta, double x_start, double y_start, double delta_t);
	double dynamic_response(double k_next, double k, double v_next, double v, double t, 
		double &k_response, double & v_response);
	bool speed_logic_control(double k_next, double v_next, double v, double &v_response);
	std::vector<RoadPoint> BestPathSelector(const std::vector<std::vector<RoadPoint>> cadidatePath);
	bool check_circle_area_collision(VeloGrid_t & grids, std::vector<RoadPoint> localPath);
	void gps_tracking();

	void short_time_several_lanes();

	void short_time_uturn();

	std::vector<std::vector<RoadPoint>> generateTrajectories(std::vector<RoadPoint>& path,bool local=false);
	void plan();
	void planJoint();
	std::vector<std::vector<RoadPoint>> planRef(std::vector<RoadPoint>& baseFrame);
	std::vector<std::vector<RoadPoint>> planRefWithSegment(std::vector<RoadPoint>& baseFrame);
	std::vector<std::vector<RoadPoint>> planRefInUTurn(std::vector<RoadPoint>& baseFrame);
	std::vector<RoadPoint> selectBestTraj(std::vector<std::vector<RoadPoint>>& paths,std::vector<RoadPoint>& prePath, std::vector<RoadPoint>& refPath);
	void SendPath(std::vector<RoadPoint>& ref, std::vector<RoadPoint>& best, std::vector<std::vector<RoadPoint>>& paths);
private:
	std::vector<RoadPoint> pre_Root;
	std::vector<RoadPoint> best_root;
	std::vector<RoadPoint> Root_On_Gaussian;
	std::vector<std::vector<PosPoint>> posPtOnRoot;
	std::vector<std::vector<RoadPoint>> clothoidMap;
	double pre_qf;
	double PreDirection;
	double target_X;
	double target_Y;
	//double car_orientation;
	PosPoint car;
	std::queue<double> recAngle;
	std::vector<float> x_ref;
	std::vector<float> y_ref;
	int CheckCollision(VeloGrid_t& grids, std::vector<RoadPoint>& localPath, bool hasAngle = false);

	int CheckCollision_2(VeloGrid_t& grids, std::vector<RoadPoint>& localPath, bool hasAngle = false);
	bool DetectGridObs(RoadPoint cur, VeloGrid_t & grids, RoadPoint &collisionpoint);

	bool m_isSegmentMode;
	float m_testVelocityFactor;

	std::vector<RoadPoint> pre_r;

	bool updatetra;
	std::vector<RoadPoint> SelectTra(std::vector<std::vector<RoadPoint>>& paths, std::vector<RoadPoint>& prePath, std::vector<RoadPoint>& refPath, double& min_maxDis);
	bool UpdateOrNot(std::vector<RoadPoint>& curPath, std::vector<RoadPoint>& prePath, std::vector<RoadPoint>& refPath);

	bool isChangingLane;
};


#endif