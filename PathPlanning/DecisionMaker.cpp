#include "DecisionMaker.h"
#include "DataCenter.h"
#include "lcmtype/VeloGrid_t.hpp"
#include "CollisionDetection.h"
#include "Topology.h"
#include "LoopThread.h"
#include "CoordTransform.h"
#include "CarControl.h"
#include "CoordTransform.h"
#include <cmath>
#include <chrono>

DecisionMaker::DecisionMaker()
{
	//default we execute the program with obstacle avoiding mode 
	m_scene = Scene::Avoidance;
	m_isSegmentMode = false;
	m_track.Start();
	m_recommendSpeed = -1.0;
	m_isUTurn = true;//for u-turn test, delete it later
	m_safeDistance = 30.0;//安全距离预设为20m，之后可能会改
	m_targetSpeed[0] = 10.0;
	m_targetSpeed[1] = 5.0;
	m_targetSpeed[2] = 3.0;
	m_targetSpeed[3] = 0.0;

	m_slowFlag = false;
	m_accer = 0.0;

	m_logFile.open("logFile_10.txt", std::ios::out);
	m_logFile << "Distance\t" << "Acceler\t" << "CurSpeed\t" << "TarSpeed" << std::endl;

	m_speedMax = 30.0;//将最大行驶速度设定为30km/h
	
}


DecisionMaker::~DecisionMaker()
{
	m_logFile.close();
}


void DecisionMaker::Execute()
{

	while (true)
	{
		std::cout << "--------New Decision Loop--------" << std::endl;
		auto startTime = std::chrono::steady_clock::now();
		m_recommendSpeed = -1.0;
		if (!DataCenter::GetInstance().WaitForVeloGrid(20)) {
			//std::cout << "No VeloGrid!!" << std::endl;
			continue;
		}
		//multilane
		if (DataCenter::GetInstance().HasMultiLane())
		{
			m_scene = Scene::MultiLine;
			
		}
		//traffic Light

		if (DataCenter::GetInstance().HasSceneMsg())
		{
			m_scene = Scene::StopLine;
		}

		makeDecision();
		m_scene = Scene::Avoidance;

		auto endTime = std::chrono::steady_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		std::cout << "Time Cost: " << duration.count() << std::endl;
		std::cout << "--------End Decision Loop---------" << std::endl;
		__thread_sleep_for(10);
	}
}

bool DecisionMaker::updateTrajectory(std::vector<RoadPoint>& curPath, std::vector<RoadPoint>& prePath, double preFactor, double curFactor)
{
	//update or not
	VeloGrid_t velol = DataCenter::GetInstance().GetLidarData();
	int collision = CollisionDetection::collisionCheck(velol, prePath, true);
	double pre_length = Topology::CalLineLength(prePath);
	double cur_length = Topology::CalLineLength(curPath);
	//last path empty or has collison or in segment mode or too short
	m_isSegmentMode = m_pathPlanner.IsSegment();
	if (prePath.empty() || collision != -1 || pre_length <= 0.6*cur_length || m_isSegmentMode || pre_length < 25)
	{
		return true;
	}
	else if (!prePath.empty())
	{
		std::cout << "last path distance to ref path : " << preFactor << std::endl;
		std::cout << "this path distance to ref path : " << curFactor << std::endl;

		if (preFactor > curFactor)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return true;
	}
}

void DecisionMaker::makeDecision()
{

	//detect stopline
	if (m_scene == Scene::StopLine)
	{
		std::cout << "StopLine Detected" << std::endl;
		Scene_t stopLineMsg = DataCenter::GetInstance().GetSceneMessage();
		if (stopLineMsg.IsBlock)
		{
			CarControl::GetInstance().StopCommand();
			//m_track.Stop();
			m_recommendSpeed = 0.0;

			//clear path to stop
			m_plannedRoot.swap(std::vector<RoadPoint>());
			m_preRoot.swap(std::vector<RoadPoint>());
			m_preRootLocal.swap(std::vector<RoadPoint>());
			SendInfo(m_plannedRoot, m_recommendSpeed);//send empty path to stop the car
			return;
		}
		else
		{
			m_recommendSpeed = stopLineMsg.velocity;
		}
		
	}
	//red light, stop car
	if (m_scene == Scene::TrafficLight)
	{
		std::cout << "Red Light, Stop!" << std::endl;
		CarControl::GetInstance().StopCommand();
		
		return;
	}

	//m_track.ReStart();
	//m_baseFrame = DataCenter::GetInstance().GetRefTrajectories();
	int laneI = 0;//number of lanes, default 0
	std::vector <std::vector<RoadPoint>> baseFrames = DataCenter::GetInstance().GetMultiLanes(laneI);
	if (!baseFrames.empty())
	{
		m_baseFrame = baseFrames[0];
	}
	else
	{
		std::cout << "没有参考路径" << std::endl;
		CarControl::GetInstance().StopCommand();
		return;
	}
	m_baseFrameLocal = BaseFrame2Local(m_baseFrame);

	//判断是否需要跟车
	double dis = 0.0;//当前位置到障碍物距离
	bool followCar = FollowFrontCar(m_safeDistance, dis);
	double curSpeed = DataCenter::GetInstance().GetCarInfo().speed;//获取当前车速
	double finalSpeed;
	if (followCar)
	{	
		std::cout << "\n处于跟车状态！！！" << std::endl;
		m_logFile << m_safeDistance << "\t" << m_accer << "\t" << curSpeed << "\t" << curSpeed << std::endl;

		m_plannedRoot = m_baseFrameLocal;//如果处于跟车模式，那么直接跟随参考路径
		SendInfo(m_plannedRoot);
		m_pathDraw.DrawSelected(m_plannedRoot);
		return;
	}
	else
	{	//测试时在不需要跟车是将速度逐渐减到0
		std::cout << "不跟车，正在减速！！！" << std::endl;

		m_plannedRoot = m_baseFrameLocal;//仅跟随参考路径
		
		m_preRootLocal = std::vector<RoadPoint>();

		m_disToFirstObstacle -= 4.0;//计算车头到前车的大致距离
		std::cout << "与前车距离： " << m_disToFirstObstacle << std::endl;

		double targetSpeed = SpeedRelativeDistance(m_disToFirstObstacle);
		std::cout << "加速度： " << m_accer << std::endl;

		//速度太慢则假定为车速减到0.0，直接停车
		if (targetSpeed < 3.0)
		{
			targetSpeed = 0.0;
			CarControl::GetInstance().StopCommand();
			m_plannedRoot.swap(std::vector<RoadPoint>());
			m_preRoot.swap(std::vector<RoadPoint>());
			m_preRootLocal.swap(std::vector<RoadPoint>());
		}
		std::cout << "目标车速： " << targetSpeed << std::endl;
		m_logFile << dis << "\t" << m_accer << "\t" << curSpeed << "\t" << targetSpeed << std::endl;
		//if (m_disToFirstObstacle > 20.0)
		//{
		//	finalSpeed = m_targetSpeed[0];
		//}
		//if (m_disToFirstObstacle > 15.0 && m_disToFirstObstacle <= 20.0)
		//{
		//	finalSpeed = m_targetSpeed[1];
		//}
		//if (m_disToFirstObstacle > 6.0 && m_disToFirstObstacle <= 15.0)
		//{
		//	finalSpeed = m_targetSpeed[2];
		//}
		//if (m_disToFirstObstacle < 6.0)
		//{
		//	finalSpeed = m_targetSpeed[3];
		//	m_plannedRoot.swap(std::vector<RoadPoint>());
		//	m_preRoot.swap(std::vector<RoadPoint>());
		//	m_preRootLocal.swap(std::vector<RoadPoint>());
		//	//m_plannedRoot = std::vector<RoadPoint>();
		//	//m_preRoot = std::vector<RoadPoint>();
		//	//m_preRootLocal = std::vector<RoadPoint>();
		//}
		m_pathDraw.DrawSelected(m_plannedRoot);
		SendInfo(m_plannedRoot, targetSpeed);
		return;
	}


	PosPoint curPos = DataCenter::GetInstance().GetCurPosition();

	//previous path is used in segment planning mode
	if (!m_preRootLocal.empty())
	{
		m_pathPlanner.Plan(m_preRoot, m_isUTurn);
		
		m_plannedRoot = m_pathPlanner.GetBestTrajecoty();
	}
	else 
	{
		m_pathPlanner.Plan(std::vector<RoadPoint>(), m_isUTurn);
		m_plannedRoot = m_pathPlanner.GetBestTrajecoty();
	}


	//update trajectory or not
	//update
	bool updateflag = true;
	updateflag = updateTrajectory(m_plannedRoot, m_preRootLocal, m_pathPlanner.GetPreFactor(), m_pathPlanner.GetCurFactor());
	if (updateflag)
	{
		m_preRootLocal = m_plannedRoot;
		//m_track.SetLocalPath(m_plannedRoot);//send final path to tracking part
		SendInfo(m_plannedRoot, m_recommendSpeed);
		m_preRoot = m_plannedRoot;
	}
	else
		std::cout << "Not Update Path !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


	//Draw Selected Path
	m_pathDraw.DrawSelected(m_plannedRoot);

	//transform last path into global coordinate system
	for (int i = 0; i < m_preRoot.size(); i++)
	{
		PosPoint tmp;
		CoordTransform::LocalToWorld(curPos, m_preRoot[i], &tmp);
		m_preRoot[i] = tmp;
	}
}


//display paths and selected path
void DecisionMaker::DrawPath(std::vector<std::vector<RoadPoint>>& paths, std::vector<RoadPoint>& bestPath)
{
	m_pathDraw.SendDraw(paths, bestPath);
}


//send decision result to control part,including trajectory and recommended speed
//if speed = -1.0, means we did not send recommend speed
void DecisionMaker::SendInfo(std::vector<RoadPoint>& localPath, float speed)
{
	m_track.SetRecommendSpeed(speed);
	m_track.SetLocalPath(localPath);
}

std::vector<RoadPoint> DecisionMaker::BaseFrame2Local(const std::vector<RoadPoint>& baseFrame)
{
	std::vector<RoadPoint> localBaseFrame;
	PosPoint cur = DataCenter::GetInstance().GetCurPosition();
	for (int i = 0; i < baseFrame.size(); i++)
	{
		RoadPoint tmp;
		CoordTransform::WorldToLocal(cur, baseFrame[i], &tmp);
		if (tmp.y > 0)
		{
			localBaseFrame.push_back(tmp);
		}

	}
	return localBaseFrame;
	
}

bool DecisionMaker::FollowFrontCar(const double& safe_distance, double& frontDis)
{
	PosPoint cur = DataCenter::GetInstance().GetCurPosition();

	if (m_baseFrameLocal.empty())
		return false;

	VeloGrid_t velo = DataCenter::GetInstance().GetLidarData();

	int collideIndex = CollisionDetection::collisionCheck(velo, m_baseFrameLocal, true);
	
	if (collideIndex == -1)
		return true;

	m_baseFrameLocal.erase(m_baseFrameLocal.begin() + collideIndex+1, m_baseFrameLocal.end());//将碰撞点以后的参考路径擦除
	m_disToFirstObstacle = (m_baseFrameLocal[collideIndex].x) * (m_baseFrameLocal[collideIndex].x) +
		(m_baseFrameLocal[collideIndex].y) * (m_baseFrameLocal[collideIndex].y);
	m_disToFirstObstacle = sqrt(m_disToFirstObstacle);
	//std::cout << m_disToFirstObstacle << std::endl;
	frontDis = m_disToFirstObstacle - 3.5;
	//减去4.0代表车头大概到前车的距离
	if (frontDis > safe_distance)
		return true;
	else
		return false;

}

//参数distance代表车头到前车的距离
double DecisionMaker::SpeedRelativeDistance(double distance)
{
	double curSpeed = DataCenter::GetInstance().GetCarInfo().speed;
	std::cout << "当前车速: " << curSpeed << std::endl;
	curSpeed /= 3.6;//km/h化为m/s

	//大于设定的安全距离，则保持当前速度行驶
	if (distance > m_safeDistance)
	{
		m_slowFlag = false;
		return curSpeed;
	}

	//如果确定开始减速，则先计算加速度
	if (!m_slowFlag)
	{
		m_slowFlag = true;
		//计算减速的加速度
		m_accer = 1.0 * curSpeed * curSpeed / (2 * m_safeDistance);
	}

	if (distance < 5.0)
	{
		return 0.0;
	}

	return sqrt(2 * m_accer * distance) * 3.6/*/2*/ ;//返回目标速度并将单位化成km/h

}


//calculate safe brake distance according to spped
//speed unit: km/h
//distance unit: m
double DecisionMaker::SafeDistance(double speed)
{
	double mu = 0.8;//一般情况下的摩擦系数
	double g = 9.8;//重力加速度

	speed /= 3.6; //将速度单位化为m/s

	double safeDistance = speed * speed / (2 * g * mu);

	return safeDistance + speed * 0.2;//加上大致的反应时间
}
