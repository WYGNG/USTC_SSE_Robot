#pragma once
#include "type.h"

namespace costmap 
{

	class Costmap {
	public:
		Costmap(int c);
		void caseMap1();
		void caseMap2();
		CharVector2D & get_costmap();
		void print_map();

	private:
		int nx_;
		int ny_;
		int ns_;
		int start_x_;
		int start_y_;
		int end_x_;
		int end_y_;
		CharVector2D costmap_;
	};

}