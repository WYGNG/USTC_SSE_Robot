#pragma once
#include <vector>

typedef std::vector<char> CharVector;
typedef std::vector<CharVector> CharVector2D;

typedef std::vector<int> IntVector;
typedef std::vector<IntVector> IntVector2D;

typedef std::vector<std::pair<int, int>> PathVector;

static const int POT_HIGH = 0x7fffffff;
static const unsigned char MAP_OBS = 35;
static const unsigned char FREE_SAPCE = 32;
static const unsigned char FOOTPRINTS = 111;