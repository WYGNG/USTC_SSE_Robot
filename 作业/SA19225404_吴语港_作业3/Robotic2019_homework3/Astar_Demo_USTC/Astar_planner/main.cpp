#include "pch.h"
#include <iostream>
#include <iomanip>
#include "astar.h"
#include "potential_calculator.h"
#include "costmap.h"
#include "grid_path.h"
using namespace astar_planner;
using namespace costmap;

int main()
{
	int nx = 20, ny = 20;
	int start_x = 2, start_y = 2;
	int goal_x = 17, goal_y = 16;

	Costmap *map = new Costmap(1);
	IntVector2D potential_array = *(new IntVector2D(nx, IntVector(ny, POT_HIGH)));
	PotentialCalculator *p_calc = new PotentialCalculator(nx, ny);
	AStar *planner = new AStar(p_calc, nx, ny);
	GridPath *pathmaker = new GridPath(start_x, start_y, goal_x, goal_y);
	PathVector path;

	bool got_plan = planner->calculatePotentials(map->get_costmap(), start_x, start_y, goal_x, goal_y, nx*ny*2, potential_array);
	if (got_plan) {
		map->print_map();
		std::cout << std::endl;
		for (int i = 0; i < nx; i++) {
			for (int j = 0; j < ny; j++) {
				if (potential_array[i][j] == POT_HIGH)
					std::cout << "---- ";
				else
					std::cout << std::setw(4) << potential_array[i][j] << ' ';
			}
			std::cout << std::endl;
		}
	}
	else
		std::cout << "Fail to make plan." << std::endl;

	bool got_path = pathmaker->getPath(potential_array, nx*ny * 2, path);
	if (got_path) {
		std::cout << std::endl;
		pathmaker->drawPath(path, map->get_costmap());
		map->print_map();
	}
	else
		std::cout << "Fail to get path." << std::endl;

	return 0;
}
