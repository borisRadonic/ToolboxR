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

#include "FIRFilter.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		FIRFilter::FIRFilter()
		{
			/*create input and aouput*/
			_ptrIn = Signal<std::double_t>::Factory::NewSignal("in1", BaseSignal::SignalType::Double);
			_ptrOut = Signal<std::double_t>::Factory::NewSignal("out1", BaseSignal::SignalType::Double);

			this->addInput(_ptrIn);
			this->addOutput(_ptrOut);
		}

		FIRFilter::~FIRFilter()
		{
		}

		void FIRFilter::setParameters(const std::vector<std::double_t>& coefficients, const std::string& name)
		{
			_b = coefficients;
			for (auto& b : _b)
			{
				_x.push_back(0.00);
			}
			setName(name);
			_isParamsSet = true;
		}

		double FIRFilter::process(std::double_t u)
		{
			if (_isParamsSet && (_b.size() > 0U) )
			{
				std::size_t count = _b.size() - 1;

				//skip values in _x
				//
				for (std::size_t i = 0U; i < count; i++)
				{
					_x[i] = _x[i + 1];
				}
				//update last value
				_x[count] = u;

				std::double_t val = 0.00;
				std::size_t ccount = 0U;
				for (auto& b : _b)
				{
					val = _x[count] * _b[ccount];
					count--;
					ccount++;
				}
				_y = val;
			}
			_ptrIn->set(u);
			_ptrOut->set(_y);
			return _y;
		}

		void FIRFilter::reset()
		{
			std::size_t count = _b.size();

			if (_isParamsSet && (count > 0U) )
			{
				for (size_t i = 0; i < _b.size(); i++)
				{
					_x[i] = 0.00;
				}
			}
		}

	}
}