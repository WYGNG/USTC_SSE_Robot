#include "pch.h"
#include "astar.h"

namespace astar_planner {

	bool AStar::calculatePotentials(CharVector2D &costs, int start_x, int start_y, int end_x, int end_y,
		int cycles, IntVector2D &potential) {  //cost是代价地图，potential是起点到各点的累计代价
		queue_.clear();  //清空队列
		int start_i = toIndex(start_x, start_y);  //toIndex在potential_calculator.h中定义，实际就是算像素点是在图像中的第几个点
		queue_.push_back(Index(start_i, 0));  //把当前点作为起始点，推入栈中，代价为0

		//std::fill(potential[0], potential[0]+ns_, POT_HIGH);  //用POT_HIGH(一个很大的值)初始化所有像素点的potential
		potential[start_x][start_y] = 0;  //起点potential为0

		int goal_i = toIndex(end_x, end_y);  //终点索引
		int cycle = 0;  //循环计数

		while (queue_.size() > 0 && cycle < cycles) {  //循环递归将周围点添加到队列
			Index top = queue_[0];
			std::pop_heap(queue_.begin(), queue_.end(), greater1());  //将cost最小的结点放到末尾
			queue_.pop_back();  //去掉cost最大的点

			int i = top.i;
			if (i == goal_i)
				return true;  //如果队列首元素是目标点，则返回
			int x = i % nx_, y = i / nx_;
			add(costs, potential, potential[x][y], x+1, y, end_x, end_y);
			add(costs, potential, potential[x][y], x-1, y, end_x, end_y);
			add(costs, potential, potential[x][y], x, y+1, end_x, end_y);
			add(costs, potential, potential[x][y], x, y-1, end_x, end_y);
			//把上下左右的点加入到队列中
		}

		return false;
	}

	void AStar::add(CharVector2D &costs, IntVector2D &potential, int prev_potential, int x, int y, int end_x, int end_y) {
		if (potential[x][y] < POT_HIGH)
			return;  //若已经遍历过该点则返回
		/*
		if (costs[next_i] >= lethal_cost_ && !(unknown_ && costs[next_i] == NO_INFORMATION))
			return;  //lethal_cost_致死代价，在expander中定义，默认值为253
		*/
		if (costs[x][y] == MAP_OBS)
			return;
		
		potential[x][y] = p_calc_->calculatePotential(potential, neutral_cost_, x, y, prev_potential);
		//potential[next_i] = prev_potential + (costs[next_i]+neutral_cost_)
		//neutral_cost_中立代价，在expander中定义，默认值为50
		int dx = abs(end_x - x);
		int dy = abs(end_y - y);
		//int distance = abs(end_x - x) + abs(end_y - y);  //当前点与目标点的曼哈顿距离
		//int distance = dx + dy;  //当前点与目标点的曼哈顿距离
		//int distance = abs(end_x - x) + abs(end_y - y);  //当前点与目标点的对角线距离
		//int distance = dx + dy + (1.4 - 2)*fmin(dx, dy);  //当前点与目标点的对角线距离
		//int distance = sqrt(abs(end_x - x)*abs(end_x - x)+ abs(end_y - y)*abs(end_y - y));  //当前点与目标点的欧几里得距离
		int distance = 100*sqrt(dx * dx + dy * dy);  //当前点与目标点的欧几里得距离
		int next_i = toIndex(x, y);
		//起点到当前点+当前点到终点
		queue_.push_back(Index(next_i, potential[x][y] + distance * neutral_cost_));  //队列中加入新结点，并定义其cost
		std::push_heap(queue_.begin(), queue_.end(), greater1());  //每插入一个结点就把最小代价结点放在最前面
	}

} //end namespace astar_planner
