#pragma once
#include <vector>

class ShortTimePathPlanning
{
public:
	ShortTimePathPlanning(float theta);
	~ShortTimePathPlanning();
	void GetBaseFrame(const std::vector<float>& x, const std::vector<float>& y, int num);//获取基本路径（参考路径）
	bool GenerateCandidatePath();
private:
	float _Qi;//起点的侧向偏移量
	float _Qf;//路径终点相对于基本路径的侧向偏移量
	std::vector<float> _BaseFrame_x;//基本路径的点的x坐标
	std::vector<float> _BaseFrame_y;//基本路径上点的y坐标
	int _BasePtCount;//基本路径上点的计数
	float _theta;//当前位置车头与基本路径的夹角
};

