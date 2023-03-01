#include "Derivative.h"

#include <algorithm>

using namespace DiscreteTime;

Derivative::Derivative() :Block(), _old(0.00), _y(0.00)
{
}

Derivative::~Derivative()
{
}

void Derivative::setParameters(std::double_t ts, std::double_t gain, const std::string & name)
{
	_Ts = ts;
	_gain = gain;
	setName(name);
	_isParamsSet = true;
}

void Derivative::setSaturation(std::double_t min, std::double_t max)
{
	_outMin = min;
	_outMax = max;
	_isUseSaturation = true;
}

void Derivative::reset()
{
	_old	= 0.00;
	_y		= 0.00;
}

double Derivative::process(std::double_t u)
{
	if (_isParamsSet)
	{
		_y = _gain * ((u - _old) / _Ts);
		_old = u;
		_y = std::clamp(_y, _outMin, _outMax);
		checkSaturation(_y);
	}
	return _y;
}

void Derivative::checkSaturation(std::double_t val)
{
	if (_isUseSaturation)
	{
		_saturate = ((val >= _outMax) || (val <= _outMin));
	}
}
