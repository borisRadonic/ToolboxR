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

#include "Derivative.h"

#include "Signal.h"

#include <algorithm>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{

		Derivative::Derivative() :Block(), _old(0.00), _y(0.00)
		{
			/*create input and aouput*/
			_ptrIn  = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);
	
			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
		}

		Derivative::~Derivative()
		{
		}

		void Derivative::setParameters(std::double_t ts, std::double_t gain, const std::string& name)
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
			_old = 0.00;
			_y = 0.00;
		}

		double Derivative::process(std::double_t u)
		{
			if (_isParamsSet)
			{
				_y = _gain * ((u - _old) / _Ts);
				_old = u;
				if (_isUseSaturation)
				{
					_y = std::clamp(_y, _outMin, _outMax);
					checkSaturation(_y);
				}
			}
			_ptrIn->set(u);
			_ptrOut->set(_y);
			return _y;
		}

		void Derivative::checkSaturation(std::double_t val)
		{
			if (_isUseSaturation)
			{
				_saturate = ((val >= _outMax) || (val <= _outMin));
			}
		}
	}
}