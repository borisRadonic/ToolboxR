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

#pragma once

#include <string>
#include <vector>
#include "Block.h"
#include "IIRFilterFO.h"

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{

			//Discrete time second order Low-Pass Butterworth filter 
			//
			class ButterworthLowPassI : public IIRFirstOrderFilter
			{
			public:

				ButterworthLowPassI();

				virtual ~ButterworthLowPassI();

				// omega_c - cutoff frequency in rad/s (radians per second)
				//
				void setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name);

				const std::double_t getCutoffFrequency() const
				{
					return _omega_c;
				}

			protected:

				std::double_t  _omega_c = 0.00;
				std::double_t _ts = 1.00;
			};

			//Discrete time second order Low-Pass Butterworth filter 
			//
			class ButterworthHighPassI : public IIRFirstOrderFilter
			{
			public:

				ButterworthHighPassI();

				virtual ~ButterworthHighPassI();

				// omega_c - cutoff frequency in rad/s (radians per second)
				//
				void setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name);

				const std::double_t getCutoffFrequency() const
				{
					return _omega_c;
				}

			protected:

				std::double_t  _omega_c = 0.00;
				std::double_t _ts = 1.00;
			};

		}
	}
}

