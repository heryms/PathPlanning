#include "CostFunc.h"
double similarity(std::vector<RoadPoint> pre_cur, std::vector<RoadPoint> now_cur){
	if (pre_cur.size() == 0)
		return 0;
	double diff = 0.0;
	for (int i = 0; i < pre_cur.size(); i++)
	{
		double delta_x = pre_cur[i].x - now_cur[i].x;
		double delta_y = pre_cur[i].y - now_cur[i].y;
		double delta_angle = pre_cur[i].angle - now_cur[i].angle;
		double delta_k = (pre_cur[i].k - now_cur[i].k);
		diff += delta_k;

	}
	return diff / pre_cur.size();

}
double cost(std::vector<RoadPoint> now_curve, std::vector<RoadPoint> reference_map, std::vector<double> guassian_prob){
	if (now_curve.size() == 0 && reference_map.size()==0)
	{
		return MAX_INT;
	}
	if (now_curve.size()!=reference_map.size())
	{
		return MAX_INT;
	}
	double cost_value = 0.0;
	for (int i = 0; i < now_curve.size();i++)
	{
		double delta_distance = abs(now_curve[i].x - reference_map[i].x) + abs(now_curve[i].y - reference_map[i].y);
		cost_value += (1 - guassian_prob[i])*delta_distance;
	}
	return cost_value;
}