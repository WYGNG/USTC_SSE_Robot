#include "pch.h"
#include "astar.h"
#include <math.h>
namespace astar_planner {

	bool AStar::calculatePotentials(CharVector2D &costs, int start_x, int start_y, int end_x, int end_y,
		int cycles, IntVector2D &potential) {  //cost�Ǵ��۵�ͼ��potential����㵽������ۼƴ���
		queue_.clear();  //��ն���
		int start_i = toIndex(start_x, start_y);  //toIndex��potential_calculator.h�ж��壬ʵ�ʾ��������ص�����ͼ���еĵڼ�����
		queue_.push_back(Index(start_i, 0));  //�ѵ�ǰ����Ϊ��ʼ�㣬����ջ�У�����Ϊ0

		//std::fill(potential[0], potential[0]+ns_, POT_HIGH);  //��POT_HIGH(һ���ܴ��ֵ)��ʼ���������ص��potential
		potential[start_x][start_y] = 0;  //���potentialΪ0

		int goal_i = toIndex(end_x, end_y);  //�յ�����
		int cycle = 0;  //ѭ������

		while (queue_.size() > 0 && cycle < cycles) {  //ѭ���ݹ齫��Χ����ӵ�����
			Index top = queue_[0];
			std::pop_heap(queue_.begin(), queue_.end(), greater1());  //��cost��С�Ľ��ŵ�ĩβ
			queue_.pop_back();  //ȥ��cost���ĵ�

			int i = top.i;
			if (i == goal_i)
				return true;  //���������Ԫ����Ŀ��㣬�򷵻�
			int x = i % nx_, y = i / nx_;
			add(costs, potential, potential[x][y], x+1, y, end_x, end_y);
			add(costs, potential, potential[x][y], x-1, y, end_x, end_y);
			add(costs, potential, potential[x][y], x, y+1, end_x, end_y);
			add(costs, potential, potential[x][y], x, y-1, end_x, end_y);
			//���������ҵĵ���뵽������
		}

		return false;
	}

	void AStar::add(CharVector2D &costs, IntVector2D &potential, int prev_potential, int x, int y, int end_x, int end_y) {
		if (potential[x][y] < POT_HIGH)
			return;  //���Ѿ��������õ��򷵻�
		/*
		if (costs[next_i] >= lethal_cost_ && !(unknown_ && costs[next_i] == NO_INFORMATION))
			return;  //lethal_cost_�������ۣ���expander�ж��壬Ĭ��ֵΪ253
		*/
		if (costs[x][y] == MAP_OBS)
			return;
		
		potential[x][y] = p_calc_->calculatePotential(potential, neutral_cost_, x, y, prev_potential);
		//potential[next_i] = prev_potential + (costs[next_i]+neutral_cost_)
		//neutral_cost_�������ۣ���expander�ж��壬Ĭ��ֵΪ50
		int dx = abs(end_x - x);
		int dy = abs(end_y - y);
		//int distance = abs(end_x - x) + abs(end_y - y);  //��ǰ����Ŀ���������پ���
		int distance = dx + dy;  //��ǰ����Ŀ���������پ���
		//int distance = abs(end_x - x) + abs(end_y - y);  //��ǰ����Ŀ���ĶԽ��߾���
		//int distance = dx + dy + (1.4 - 2)*fmin(dx, dy);  //��ǰ����Ŀ���ĶԽ��߾���
		//int distance = sqrt(abs(end_x - x)*abs(end_x - x)+ abs(end_y - y)*abs(end_y - y));  //��ǰ����Ŀ����ŷ����þ���
		//int distance = sqrt(dx * dx + dy * dy);  //��ǰ����Ŀ����ŷ����þ���
		int next_i = toIndex(x, y);
		queue_.push_back(Index(next_i, potential[x][y] + distance * neutral_cost_));  //�����м����½�㣬��������cost
		std::push_heap(queue_.begin(), queue_.end(), greater1());  //ÿ����һ�����Ͱ���С���۽�������ǰ��
	}

} //end namespace astar_planner
