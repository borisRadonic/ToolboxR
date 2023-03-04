#include "Integrator.h"
#include <algorithm>

namespace DiscreteTime
{

	Integrator::Integrator() :Block(), _ic(0.00), _x(0.00), _y(0.00), _method(IntegratorMethod::ForwardEuler)
	{
	}

	Integrator::~Integrator()
	{
	}

	void Integrator::setParameters(IntegratorMethod method, std::double_t ts, std::double_t gain, const std::string& name)
	{
		_method = method;
		_Ts = ts;
		_gain = gain;
		_isParamsSet = true;
		setName(name);
	}

	void Integrator::setInitialConditions(std::double_t ic)
	{
		_ic = ic;
		_x = ic;
		_y = ic;
	}

	void Integrator::setSaturation(std::double_t min, std::double_t max)
	{
		_outMin = min;
		_outMax = max;
		_isUseSaturation = true;
	}

	void Integrator::reset()
	{
		_x = _ic;
		_y = _ic;
		_u1 = 0.00;
	}

	double Integrator::process(std::double_t u)
	{
		if (_isParamsSet)
		{
			switch (_method)
			{
			case IntegratorMethod::BackwardEuler:
			{
				_y = _x + _Ts * _gain * u;
				if (_isUseSaturation)
				{
					_y = std::clamp(_y, _outMin, _outMax);
					checkSaturation(_y);
				}
				_x = _y;
				break;
			}
			case IntegratorMethod::ForwardEuler:
			{
				_x = _x + _Ts * _gain * _u1;
				if (_isUseSaturation)
				{
					_x = std::clamp(_x, _outMin, _outMax);
					checkSaturation(_x);
				}
				_y = _x;
				_u1 = u;
				break;
			}
			case IntegratorMethod::Trapezoidal:
			{
				_y = _x + _gain * (_Ts / 2.0) * u;
				if (_isUseSaturation)
				{
					_y = std::clamp(_y, _outMin, _outMax);
					checkSaturation(_y);
				}
				_x = _y + _gain * (_Ts / 2.0) * u;
				if (_isUseSaturation)
				{
					_x = std::clamp(_x, _outMin, _outMax);
					checkSaturation(_x);
				}
				break;
			}
			}
			//
		}
		return _y;
	}

	void Integrator::checkSaturation(std::double_t val)
	{
		if (_isUseSaturation)
		{
			_saturate = ((val >= _outMax) || (val <= _outMin));
		}
	}

}