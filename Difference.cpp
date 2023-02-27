#include "Difference.h"
#include <algorithm>

using namespace DiscreteTime;

Difference::Difference() :Block(), _old(0.00), _y(0.00)
{
}

Difference::~Difference()
{
}

double Difference::process(std::double_t u)
{
	_y = u - _old;
	_old = u;
	return _y;
}
