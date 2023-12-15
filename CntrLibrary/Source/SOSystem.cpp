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

#include "SOSystem.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{

		SOSystem::SOSystem()
		{
			/*create input and aouput*/
			_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
		}


		SOSystem::~SOSystem()
		{
		}

		void SOSystem::setParameters(const std::double_t a1, const std::double_t a2, const std::double_t b0, const std::double_t b1, const std::double_t b2, const std::string& name)
		{
			_a1 = a1;
			_a2 = a2;
			_b0 = b0;
			_b1 = b1;
			_b2 = b2;
			setName(name);
			_isParamsSet = true;
		}


		void SOSystem::reset()
		{
			_x0 = 0.00;
			_y0 = 0.00;
			_x1 = 0.00;
			_y1 = 0.00;
			_x2 = 0.00;
			_y2 = 0.00;
		}

		double SOSystem::process(std::double_t u)
		{
			if (_isParamsSet)
			{
				_x0 = u;
				_y0 = (_b0 * u) + (_b1 * _x1) + (_b2 * _x2) - (_a1 * _y1) - (_a2 * _y2);
				_x0 = u;
				
				_y2 = _y1;
				_y1 = _y0;
				_x2 = _x1;
				_x1 = u;
			}
			else
			{
				_y0 = u;
			}
			_ptrIn->set(u);
			_ptrOut->set(_y0);
			return _y0;
		}
	}
}

