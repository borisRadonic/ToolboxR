/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include "Integrator.h"
#include <algorithm>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		Integrator::Integrator() :Block(), _ic(0.00), _x(0.00), _y(0.00), _method(IntegratorMethod::ForwardEuler)
		{
			/*create input and aouput*/
			_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
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
			_ptrIn->set(u);
			_ptrOut->set(_y);
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
}