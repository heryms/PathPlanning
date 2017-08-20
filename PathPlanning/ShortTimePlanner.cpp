#include "ShortTimePlanner.h"
#include "CoordTransform.h"
#include "CarControl.h"
#include "CollisionDetection.h"
#include "Clothoid.h"
#include <numeric>

ShortTimePlanner::ShortTimePlanner()
{
	m_testVelocityFactor = 1.0;
	m_preFactor = m_curFactor = 0.0;
	m_isInUTurn = false;
}


ShortTimePlanner::~ShortTimePlanner()
{

}

std::vector<RoadPoint> ShortTimePlanner::GetBestTrajecoty()
{
	return m_bestTrajectory;
}

//Previous root in local coordinate system is needed in segment mode
void ShortTimePlanner::Plan(const std::vector<RoadPoint> pre_rootGlobal, bool isUTurn)
{
	m_preRoot = pre_rootGlobal;
	std::cout << "------------------Plan Start------------------------" << std::endl;
	std::vector<std::vector<RoadPoint>> baseFrames;
	int laneI = 0;//number of lanes, default 0
	if (DataCenter::GetInstance().HasMultiLane()) {
		baseFrames = DataCenter::GetInstance().GetMultiLanes(laneI);
	}
	else {
		if (DataCenter::GetInstance().HasRefTrajectory()) {
			baseFrames.push_back(DataCenter::GetInstance().GetRefTrajectories());
		}
		else {
			std::cout << "没有参考路径" << std::endl;
			return;
		}
	}
	//init last path in this relative coordinate system
	PosPoint curPos = DataCenter::GetInstance().GetCurPosition();
	std::vector<RoadPoint> path_tmp;
	int collision;
	if (!m_preRoot.empty())
	{
		int indexI = -1;
		for (PosPoint& rpt : m_preRoot)
		{
			CoordTransform::WorldToLocal(curPos, rpt, &rpt);
			if (rpt.y < 0)
				indexI++;
		}
		if (indexI >= 0)
			m_preRoot.erase(m_preRoot.begin(), m_preRoot.begin() + indexI);
	}

	double best_cost = DBL_MAX;
	std::vector<RoadPoint> best_baseframe;
	std::vector<std::vector<RoadPoint>> candidateTrajs;

	for (int laneIndex = 0; laneIndex < baseFrames.size();laneIndex++)
		//for (int laneIndex = laneI; laneIndex < baseFrames.size(); laneIndex = (++laneIndex) >= baseFrames.size() ? (0) : ((laneIndex == laneI) ? baseFrames.size() : laneIndex)) 
	{
		std::vector<RoadPoint> baseFrame = baseFrames[laneIndex];
		//transform baseframe coord into relative coordinate
		for (RoadPoint& rpt : baseFrame) {
			CoordTransform::WorldToLocal(curPos, rpt, &rpt);
		}
		std::vector<std::vector<RoadPoint>> tmp_candidateTraj = planWithRef(baseFrame);
		if (tmp_candidateTraj.empty()) {
			std::cout << "纯路径走不了" << std::endl;
			CarControl::GetInstance().StopCommand();
			bool onUTurn = false;
			if (isUTurn) {
				/*tmp_candidateTraj = planRefInUTurn(baseFrame);
				if (tmp_candidateTraj.empty()) {
					std::cout << "调不了头" << std::endl;
					CarControl::GetInstance().StopCommand();
					continue;
				}*/
				tmp_candidateTraj = planWithUTurn(baseFrame);
				if (tmp_candidateTraj.empty())
				{
					std::cout << "未寻找到可执行掉头路径" << std::endl; 
					CarControl::GetInstance().StopCommand();
					continue;
				}
			}
			else {
				tmp_candidateTraj = planWithSegment(baseFrame);
				if (tmp_candidateTraj.empty()) {
					std::cout << "无路径" << std::endl;
					CarControl::GetInstance().StopCommand();
					continue;
				}
			}
		}

		candidateTrajs.insert(candidateTrajs.end(), tmp_candidateTraj.begin(), tmp_candidateTraj.end());

		double tmp_cost = 0.0;
		//m_bestTrajectory
		std::vector<RoadPoint> bestRoot = SelectBestTrajectory(tmp_candidateTraj, m_preRoot, baseFrame, tmp_cost);
		

		if (best_cost > tmp_cost)
		{
			best_cost = tmp_cost;
			best_baseframe = baseFrame;
			m_bestTrajectory = bestRoot;
		}
	}


	m_preFactor = Topology::CalculateFinalDisTOGPS(m_preRoot, best_baseframe);
	m_curFactor = Topology::CalculateFinalDisTOGPS(m_bestTrajectory, best_baseframe);
	
	////update
	//bool updateflag = true;
	//updateflag = updateTrajectory(best_bestRoot, m_preRoot, best_baseframe);
	//if (updateflag)
	//{
	//	m_preRootLocal = best_bestRoot;
	//	//track.SetLocalPath(best_bestRoot);
	//	m_preRoot = best_bestRoot;
	//}
	//else
	//	std::cout << "Not Update Path !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

	////SendPath(best_baseframe, best_bestRoot, candidateTrajs);

	////transform last path into global coordinate system
	//for (int i = 0; i < m_preRoot.size(); i++)
	//{
	//	PosPoint tmp;
	//	CoordTransform::LocalToWorld(curPos, m_preRoot[i], &tmp);
	//	m_preRoot[i] = tmp;
	//}
	m_pathsDraw.DrawCandidates(candidateTrajs);//将所有规划出的备选路径绘制出来
	std::cout << "------------------Plan End------------------------" << std::endl << std::endl;
	return;
}

std::vector<RoadPoint> ShortTimePlanner::trajectory_build(float qf, float qi, float theta, double sf, SXYSpline & spline)
{
	/*return std::vector<RoadPoint>();*/
	std::vector<RoadPoint> pts;
	float c = tan(theta);
	float a = 2 * (qi - qf) / (pow(sf, 3)) + c / pow(sf, 2);
	float b = 3 * (qf - qi) / (pow(sf, 2)) - 2 * c / sf;
	bool firstObstacle = true;

	for (int i = 0; i <= 50; i++)
	{
		double delta_s = sf / 50 * i;
		m_testVelocityFactor = 1.0;
		double temp = m_testVelocityFactor * delta_s;
		if (temp > sf)
			temp = sf;

		float q = a * pow(temp, 3) + b * pow(temp, 2) + c * temp + qi;
		double x = 0.0, y = 0.0;
		spline.getXY(delta_s, x, y);
		double _delta_x_, _delta_y_;
		spline.getDeriveXY(delta_s, _delta_x_, _delta_y_);
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
		pts.push_back(tmp);
	}
	return pts;
}

std::vector<std::vector<RoadPoint>> ShortTimePlanner::generateTrajectories(std::vector<RoadPoint>& baseFrame, bool local)
{
	//return std::vector<std::vector<RoadPoint>>();
	std::vector<std::vector<RoadPoint>> trajectories;
	if (!local) {
		PosPoint curPos = DataCenter::GetInstance().GetCurPosition();
		for (RoadPoint& rpt : baseFrame) {
			CoordTransform::WorldToLocal(curPos, rpt, &rpt);
		}
	}
	VeloGrid_t veloGrids = DataCenter::GetInstance().GetLidarData();
	double qi = CoordTransform::TrimLocalPathToCurPt(baseFrame);
	double theta = PI / 2 - baseFrame.begin()->angle;
	SXYSpline spline;
	spline.init(baseFrame);
	if (spline.splineNum <= 4)
		return trajectories;
	double Sf = *spline.S.rbegin();
	baseFrame.clear();
	//将BaseFrame等间隔划分为50个点
	for (int i = 0; i <= 50; i++) {
		double deltaS = i*Sf / 50;
		RoadPoint rpt;
		spline.getXY(deltaS, rpt.x, rpt.y);
		double deltaX, deltaY;
		spline.getDeriveXY(deltaS, deltaX, deltaY);
		rpt.angle = atan2(deltaY, deltaX);
		baseFrame.push_back(rpt);
	}
	for (double step = 0; step < 12; step += 0.2) {
		double qf = step - 6;
		std::vector<RoadPoint> trajectory = trajectory_build(qf, qi, theta, *spline.S.rbegin(), spline);
		//collision detection
		int findOb = CollisionDetection::collisionCheck(veloGrids, trajectory, true);
		/*int findOb = collisionCheck(veloGrids, trajectory, true);*/
		if (findOb < 0) {
			trajectories.push_back(trajectory);
		}
	}
	return trajectories;
}

//plan to the goal position directly
std::vector<std::vector<RoadPoint>> ShortTimePlanner::planWithRef(std::vector<RoadPoint>& baseFrame)
{
	m_isSegmentMode = false;
	m_testVelocityFactor = 1.5;
	std::vector<std::vector<RoadPoint>> candidateTraj = generateTrajectories(baseFrame, true);
	return candidateTraj;
}

//when the obstacle is close enough to the car, planning with baseframe will fail, so we
//detect the first collision point, and change the goal to the point behind the collided point
std::vector<std::vector<RoadPoint>> ShortTimePlanner::planWithSegment(std::vector<RoadPoint>& baseFrame)
{
	m_isSegmentMode = true;

	m_testVelocityFactor = 0.8;
	std::vector<std::vector<RoadPoint>> candidateTraj;
	int count = 0;
	do
	{
		VeloGrid_t veloGrids = DataCenter::GetInstance().GetLidarData();
		std::vector<RoadPoint> searchRoot;
		if (m_preRootLocal.empty()) {
			searchRoot = baseFrame;
		}
		else {
			searchRoot = m_preRootLocal;
		}
		double qi = CoordTransform::TrimLocalPathToCurPt(searchRoot);
		SXYSpline spline;
		spline.init(searchRoot);
		if (spline.splineNum <= 4) 
			break;
		double Sf = *spline.S.rbegin();
		searchRoot = trajectory_build(0, qi, PI / 2 - searchRoot.begin()->angle, Sf, spline);
		int findOb = CollisionDetection::collisionCheck(veloGrids, searchRoot, true);
		if (findOb >= 0) {
			findOb = (findOb + 2) < spline.S.size() ? findOb + 2 : spline.S.size();
			searchRoot.erase(searchRoot.begin() + findOb, searchRoot.end());
		}
		if (searchRoot.size() <= 5) 
			break;
		candidateTraj = generateTrajectories(searchRoot, true);
		if (!candidateTraj.empty()) {
			double deltaS = Sf*findOb / 50;
			qi = CoordTransform::TrimLocalPathToCurPt(baseFrame);
			spline.init(baseFrame);
			baseFrame.clear();
			for (int i = 0; i <= 50; i++) {
				RoadPoint rpt;
				spline.getXY(deltaS*i / 50, rpt.x, rpt.y);
				double dx, dy;
				spline.getDeriveXY(deltaS*i / 50, dx, dy);
				rpt.angle = atan2(dy, dx);
				baseFrame.push_back(rpt);
			}
			break;
		}
	} while (count++ < 10);
	return candidateTraj;
}

std::vector<std::vector<RoadPoint>> ShortTimePlanner::planWithUTurn(std::vector<RoadPoint>& baseFrame)
{
	RadAngle go_angle = baseFrame[0].angle;
	RadAngle go_angle0 = go_angle + 0.2;
	RadAngle go_angle1 = go_angle - 0.2;
	RadAngle back_angle = 2 * PI - go_angle;
	RadAngle back_angle0 = back_angle + 0.2;
	RadAngle back_angle1 = back_angle - 0.2;
	int back_num = 0;
	int go_num = 0;
	int back_start = -1;
	int go_end = -1;
	for (int j = 0; j < baseFrame.size(); j++) {
		if (go_end < 0) {
			if (!baseFrame[j].angle.belong(go_angle1, go_angle0)) {
				go_num++;
				if (go_num >= 5) {
					go_end = j;
				}
			}
			else {
				go_num = 0;
			}
		}
		else {
			if (baseFrame[j].angle.belong(back_angle1, back_angle0)) {
				back_num++;
				if (back_num >= 5) {
					back_start = j;
					break;
				}
			}
			else {
				back_num = 0;
			}
		}
	}
	if (back_start < 0) {
		std::cout << "未找到掉头" << std::endl;
		return planWithSegment(baseFrame);
	}
	else {
		std::cout << "发现掉头" << std::endl;
		VeloGrid_t veloGrids = DataCenter::GetInstance().GetLidarData();
		for (int i = 0; i < go_end; i++) {
			PosPoint startPt = baseFrame[go_end - i];
			int searchSt = back_start - 3;
			for (int j = 0; j < 6; j++) {
				std::vector<RoadPoint> tmpRef(baseFrame.begin(), baseFrame.begin() + go_end - i);
				tmpRef.insert(tmpRef.end(), baseFrame.begin() + searchSt + 1, baseFrame.end());
				PosPoint target = baseFrame[searchSt + j];
				Clothoid clothoid(startPt.x, startPt.y, startPt.angle, target.x, target.y, target.angle);
				std::vector<RoadPoint> pts(10, RoadPoint());
				clothoid.PointsOnClothoid(pts, 10);
				int findob = CollisionDetection::collisionCheck(veloGrids, pts, false);
				if (findob < 0) {
					tmpRef.insert(tmpRef.begin() + go_end - i, pts.begin() + 1, pts.end() - 1);
					std::vector<std::vector<RoadPoint>> candicatePaths = planWithRef(tmpRef);
					if (candicatePaths.empty()) {
						return planWithSegment(tmpRef);
					}
					return candicatePaths;
				}
			}
			back_start++;
		}
	}
	return std::vector<std::vector<RoadPoint>>();
}

std::vector<std::vector<RoadPoint>> ShortTimePlanner::planWithUTurn1(std::vector<RoadPoint>& baseFrame, bool isLocal)
{
	if (!isLocal)
	{
		PosPoint curPos = DataCenter::GetInstance().GetCurPosition();
		//convert global coordinate to local
		for (PosPoint& rpt : baseFrame)
		{
			CoordTransform::WorldToLocal(curPos, rpt, &rpt);
		}
	}
	RoadPoint uturnPoint;
	int index;
	bool isFind = false;
	for (int i = 0; i < baseFrame.size() - 1; i++)
	{
		//如果找到一个位置，下一个点的y坐标开始下降，则认为该点是u-turn的最远点
		if (baseFrame[i].y > baseFrame[i + 1].y)
		{
			uturnPoint = baseFrame[i];
			isFind = true;
			index = i;
			break;
		}
	}
	std::vector<std::vector<RoadPoint>> candidates;
	if (isFind)
	{
		baseFrame.erase(baseFrame.begin() + index, baseFrame.end());
		candidates = planWithRef(baseFrame);
	}
	else
	{
		candidates = planWithSegment(baseFrame);
	}
	
	return candidates;
	//return std::vector<std::vector<RoadPoint>>();
}

std::vector<RoadPoint> ShortTimePlanner::SelectBestTrajectory(std::vector<std::vector<RoadPoint>>& paths, std::vector<RoadPoint>& prePath, std::vector<RoadPoint>& refPath, double & min_maxDis)
{
	//return std::vector<RoadPoint>();
	RoadPoint curPt = DataCenter::GetInstance().GetCurPosition();
	//init last path in this relative coordinate system
	//std::vector<RoadPoint> pre_r_cur(prePath.begin(), prePath.end());
	std::vector<RoadPoint> path_tmp;
	int collision;
	if (!prePath.empty())
	{
		VeloGrid_t velol = DataCenter::GetInstance().GetLidarData();
		collision = CollisionDetection::collisionCheck(velol, prePath, true);

		if (collision == -1 && m_isSegmentMode)
		{
			std::cout << "与上一条路径比较！！！" << std::endl;
			path_tmp = prePath;
		}
		else
		{
			path_tmp = refPath;
		}
	}
	else
	{
		path_tmp = refPath;
	}
	//select best trajectory in this term
	std::vector<double > disIndex;
	std::vector<double> disIndex2;
	for (int i = 0; i < paths.size(); i++)
	{
		double DisI = Topology::CalculateDisTOGPS(paths[i], path_tmp);
		//double DisI = CalculateDisTOGPS(paths[i], path_tmp);
		disIndex.push_back(DisI);
		double dissum = Topology::CalculateSumDisToGPS(paths[i], path_tmp);
		disIndex2.push_back(dissum);
	}

	double fullenergy = std::accumulate(disIndex2.begin(), disIndex2.end(), 0.00000001);
	std::vector<double> disindex3;
	for (int i = 0; i < disIndex.size();i++)
	{
		double val = 0.3 * disIndex[i] + 0.7 * disIndex2[i] / fullenergy;
		disindex3.push_back(val);
	}

	double aaa = DBL_MAX;
	int index = 0;
	for (int u = 0; u < disindex3.size(); u++)
	{
		if (aaa > disindex3[u])
		 {
			aaa = disindex3[u];
			index = u;
		}
	}
	min_maxDis = Topology::CalculateAveDisTOGPS(paths[index], path_tmp);
	return paths[index];
}

bool ShortTimePlanner::updateTrajectory(std::vector<RoadPoint>& curPath, std::vector<RoadPoint>& prePath, std::vector<RoadPoint>& refPath)
{
	//update or not
	VeloGrid_t velol = DataCenter::GetInstance().GetLidarData();
	int collision = CollisionDetection::collisionCheck(velol, prePath, true);
	double pre_length = Topology::CalLineLength(prePath);
	double cur_length = Topology::CalLineLength(curPath);
	//last path empty or has collison or in segment mode or too short
	if (prePath.empty() || collision != -1 || pre_length <= 0.6*cur_length || m_isSegmentMode || pre_length < 25)
	{
		return true;
	}
	else if (!prePath.empty())
	{
		double pre_factor = Topology::CalculateFinalDisTOGPS(prePath, refPath);
		double ths_factor = Topology::CalculateFinalDisTOGPS(curPath, refPath);
		std::cout << "last path distance to ref path : " << pre_factor << std::endl;
		std::cout << "this path distance to ref path : " << ths_factor << std::endl;

		if (pre_factor > ths_factor)
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

bool ShortTimePlanner::IsSegment()
{
	return m_isSegmentMode;
}

double ShortTimePlanner::GetPreFactor()
{
	return m_preFactor;
}
double ShortTimePlanner::GetCurFactor()
{
	return m_curFactor;
}

