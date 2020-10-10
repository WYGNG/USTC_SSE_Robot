#include "pch.h"
#include "grid_path.h"

namespace astar_planner
{
	bool GridPath::getPath(IntVector2D & potential, int cycles, PathVector &path) {

		std::pair<int, int> current;
		current.first = end_x_;
		current.second = end_y_;
		path.push_back(current);
		int c = 0;

		while (current.first != start_x_ || current.second != start_y_) {
			int min_val = POT_HIGH;
			int min_x = 0, min_y = 0;

			for (int dx = -1; dx <= 1; dx++) {
				for (int dy = -1; dy <= 1; dy++) {
					if (dx == 0 && dy == 0)
						continue;
					int x = current.first + dx, y = current.second + dy;
					if (potential[x][y] < min_val) {
						min_val = potential[x][y];
						min_x = x;
						min_y = y;
					}
				}
			}

			if (min_x == 0 && min_y == 0)
				return false;
			if (c++ > cycles)
				return false;
			current.first = min_x;
			current.second = min_y;
			path.push_back(current);
		}

		return true;
	}

	void GridPath::drawPath(PathVector &path, CharVector2D &map) {
		if (path.size() == 0 || path.size() == 1)
			return;
		for (int i = 1; i < path.size() - 1; i++) {
			map[path[i].first][path[i].second] = FOOTPRINTS;
		}
	}
}