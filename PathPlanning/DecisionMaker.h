#pragma once

#include "BaseType.h"
#include <vector>
#include "ShortTimePlanner.h"
#include "PathDraw.h"
#include "TrackHelper.h"
#include <fstream>

enum Scene
{
	Avoidance = 0,//避障模式
	TrafficLight,//交通灯检测为红灯
	StopLine,//前方停止线
	MultiLine,//多车道
	Pedestrians,//人行道
	Follow，	//跟随前车行驶
};

class DecisionMaker
{
public:
	DecisionMaker();
	~DecisionMaker();
	//return true if make a decision successfully, default return true
	void Execute();
	bool updateTrajectory(std::vector<RoadPoint>& curPath, std::vector<RoadPoint>& prePath, double preFactor, double curFactor);
	void DrawPath(std::vector<std::vector<RoadPoint>>& paths, std::vector<RoadPoint>& bestPath);
	void SendInfo(std::vector<RoadPoint>& localPath, float speed = -1.0);
	std::vector<RoadPoint> BaseFrame2Local(const std::vector<RoadPoint>& baseFrame);//将参考路径转换到相对于车的坐标，并将车后方的点删除
	
	bool FollowFrontCar(const double& safe_distance, double& frontDis);//根据安全距离来判定是否跟随前车
	double SpeedRelativeDistance(double distance);//根据车子和前面障碍物的距离来返回相应速度
	double SafeDistance(double speed);//根据车速来计算跟车的安全距离

protected:
	void makeDecision();
private:
	std::vector<RoadPoint> m_preRootLocal;//previous path stored in local coordinate system
	std::vector<RoadPoint> m_preRoot;//previous path, stored in global coordinate system
	std::vector<RoadPoint> m_plannedRoot;//trajectory planned by ShortTimePlanner
	std::vector<RoadPoint> m_baseFrame;
	std::vector<RoadPoint> m_baseFrameLocal;//相对于车体坐标的参考路径
	double m_safeDistance;//safe distance, if short than this distance, we change the lane, do not follow the car
	double m_disToFirstObstacle;//the distance to the first obstacle collides with the baseFrame
	Scene m_scene;//some common scene
	bool m_isSegmentMode;
	ShortTimePlanner m_pathPlanner;

	PathDraw m_pathDraw;
	TrackHelper m_track;
	double m_recommendSpeed;//recommend speed
	bool m_isUTurn;
	double m_targetSpeed[4];
	bool m_slowFlag;//标志正在减速
	double m_accer;//减速所需要的加速度
	double m_speedMax;//行驶的最大速度 km/h为单位
	std::fstream m_logFile;
};

