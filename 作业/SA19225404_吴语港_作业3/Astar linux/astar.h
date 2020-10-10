#pragma once
#include "potential_calculator.h"
#include "type.h"
#include <vector>
#include <algorithm>
//#define POT_HIGH 0x7fffffff

namespace astar_planner {

	class Index {  //Index结构体，包含一个序列号和一个代价
	public:
		Index(int a, int b) {
			i = a;
			cost = b;
		}
		int i;
		int cost;
	};

	struct greater1 {
		bool operator()(const Index& a, const Index& b) const {
			return a.cost > b.cost;
		}
	};


	class AStar {
	public:
		AStar(PotentialCalculator* p_calc, int nx, int ny) : neutral_cost_(50), p_calc_(p_calc) {
			setSize(nx, ny);
		}
		bool calculatePotentials(CharVector2D &costs, int start_x, int start_y, int end_x, int end_y, int cycles,
			IntVector2D &potential);

		virtual void setSize(int nx, int ny) {
			nx_ = nx;
			ny_ = ny;
			ns_ = nx * ny;
		}
		
	private:
		inline int toIndex(int x, int y) {
			return x + nx_ * y;
		}

		int nx_, ny_, ns_;
		unsigned char neutral_cost_;
		PotentialCalculator* p_calc_;

		void add(CharVector2D &costs, IntVector2D &potential, int prev_potential, int x, int y, int end_x, int end_y);
		std::vector<Index> queue_;

	};

} //end namespace astar_planner
