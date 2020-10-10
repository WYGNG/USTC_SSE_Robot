#pragma once
#include "type.h"
#include <algorithm>

namespace astar_planner {

	class PotentialCalculator {
	public:
		PotentialCalculator(int nx, int ny) {
			setSize(nx, ny);
		}

		int calculatePotential(IntVector2D &potential, int cost, int x, int y, int prev_potential = -1) {
			if (prev_potential < 0) {
				// get min of neighbors
				int min_h = std::min(potential[x][y-1], potential[x][y+1]),
					min_v = std::min(potential[x+1][y], potential[x-1][y]);
				prev_potential = std::min(min_h, min_v);
			}
			return prev_potential + cost;
		}

		void setSize(int nx, int ny) {
			nx_ = nx;
			ny_ = ny;
			ns_ = nx * ny;
		} /**< sets or resets the size of the map */

	protected:
		inline int toIndex(int x, int y) {
			return x + nx_ * y;
		}

		int nx_, ny_, ns_; /**< size of grid, in pixels */
	};

} //end namespace astar_planner