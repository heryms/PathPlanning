#include "ShortTimePathPlanning.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>

ShortTimePathPlanning::ShortTimePathPlanning(float theta):
	_Qi(0.0), _Qf(0.0), _BasePtCount(0), _theta(theta)
{
}


ShortTimePathPlanning::~ShortTimePathPlanning()
{
	
}

void ShortTimePathPlanning::GetBaseFrame(const std::vector<float>& x, const std::vector<float>& y, int num)
{
	_BasePtCount = num;
	for (int i = 0; i < num; i++)
	{
		_BaseFrame_x.push_back(x.at(i));
		_BaseFrame_y.push_back(y.at(i));
	}
}

bool ShortTimePathPlanning::GenerateCandidatePath()
{
	Eigen::MatrixXf s = Eigen::MatrixXf::Zero(1, _BasePtCount);
	Eigen::MatrixXf accumulate_s = s;
	Eigen::MatrixXf dx = Eigen::MatrixXf::Zero(1, _BasePtCount);
	Eigen::MatrixXf dy = Eigen::MatrixXf::Zero(1, _BasePtCount);
	for (int i = 1; i < _BasePtCount; i++)
	{
		dx(0, i) = _BaseFrame_x[i] - _BaseFrame_x[i - 1];
		dy(0, i) = _BaseFrame_y[i] - _BaseFrame_y[i - 1];
		s(0, i) = sqrt(dx(0, i) * dx(0, i) +
			dy(0, i) * dy(0, i));
		accumulate_s(0, i) = s(0, i) + s(0, i - 1);
	}

	float a, b, c;//q(s)表达式系数
	//a = 2 * (qi - qf) / (s(1, 101) ^ 3) + c / (s(1, 101) ^ 2);
	//b = 3 * (qf - qi) / (s(1, 101) ^ 2) - 2 * c / (s(1, 101));
	//c = tan(theta)
	c = tan(_theta);
	float sf = accumulate_s(0, _BasePtCount - 1);//整条路径长度
	a = 2 * (_Qi - _Qf) / (sf * sf * sf) + c / (sf * sf);
	b = 3 * (_Qf - _Qi) / (sf * sf) - 2 * c / sf;
	
	Eigen::MatrixXf result_x = Eigen::MatrixXf::Zero(_BasePtCount, 1);
	Eigen::MatrixXf result_y = Eigen::MatrixXf::Zero(_BasePtCount, 1);
	Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(_BasePtCount, 1);
	for (int i = 1; i < _BasePtCount; i++)
	{
		float delta_s = accumulate_s(0, i);
		float tmp_q = a * pow(delta_s, 3) + b * pow(delta_s, 2) + c * delta_s + _Qi;
		Q(i, 0) = tmp_q;
	}

	return false;
}
