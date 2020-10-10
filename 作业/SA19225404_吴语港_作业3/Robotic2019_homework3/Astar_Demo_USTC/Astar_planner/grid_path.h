#pragma once
#include "type.h"
#include <algorithm>

namespace astar_planner 
{

	class GridPath {

	public:
		GridPath(int start_x, int start_y, int end_x, int end_y) : start_x_(start_x), start_y_(start_y), end_x_(end_x), end_y_(end_y) {}

		bool getPath(IntVector2D & potential, int cycles, PathVector &path);
		void drawPath(PathVector &path, CharVector2D &map);

	private:
		int start_x_;
		int start_y_;
		int end_x_;
		int end_y_;

	};

}