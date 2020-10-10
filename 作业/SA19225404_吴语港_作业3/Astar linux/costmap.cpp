#include "pch.h"
#include "costmap.h"
#include <iostream>

namespace costmap {

	Costmap::Costmap(int c) :
		nx_(0), ny_(0), start_x_(0), start_y_(0), end_x_(0), end_y_(0), ns_(0), costmap_(NULL) {
		if (c == 1)
			caseMap1();
		else if (c == 2)
			caseMap2();
	}

	void Costmap::caseMap1() {
		nx_ = 20;
		ny_ = 20;
		ns_ = nx_ * ny_;
		start_x_ = 2;
		start_y_ = 2;
		end_x_ = 17;
		end_y_ = 16;
		costmap_ = *(new CharVector2D(nx_, CharVector(ny_)));
		for (int i = 0; i < nx_; i++) {
			for (int j = 0; j < ny_; j++) {
				if (i == 0 || i == nx_ - 1 || j == 0 || j == ny_ - 1)
					costmap_[i][j] = MAP_OBS;
				else
					costmap_[i][j] = FREE_SAPCE;
			}
		}
		costmap_[start_x_][start_y_] = 'S';
		costmap_[end_x_][end_y_] = 'E';
		for (int i = 7, j = 6; i < nx_ - 1; i++) {
			costmap_[i][j] = MAP_OBS;
		}
		for (int i = 1, j = 14; i < 13; i++) {
			costmap_[i][j] = MAP_OBS;
		}
	}

	void Costmap::caseMap2() {}

	CharVector2D & Costmap::get_costmap() {
		return costmap_;
	}

	void Costmap::print_map() {
		for (int i = 0; i < nx_; i++) {
			for (int j = 0; j < ny_; j++) {
				std::cout << costmap_[i][j] << ' ';
			}
			std::cout << std::endl;
		}
	}

}