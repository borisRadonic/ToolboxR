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

#include "IIRFilterButterFO.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{

			/// <summary>
			/// Butterworth LP Filter 1st order
			/// </summary>
			/// 
			ButterworthLowPassI::ButterworthLowPassI()
			{
			}

			ButterworthLowPassI::~ButterworthLowPassI()
			{
			}

			void ButterworthLowPassI::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{
					std::double_t alpha_vel = exp(-omega_c * ts);
					_a1 = -alpha_vel;
					_b0 = 1.00 - alpha_vel;
					_b1 = 0.00;
					setName(name);
					_isParamsSet = true;
				}
				else
				{
					_isParamsSet = false;
				}
			}

			/// <summary>
			/// Butterworth HP Filter 1st order
			/// </summary>
			/// 
			ButterworthHighPassI::ButterworthHighPassI()
			{
			}

			ButterworthHighPassI::~ButterworthHighPassI()
			{
			}

			void ButterworthHighPassI::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{
					std::double_t alpha_vel = exp(-omega_c * ts);
					_a1 = -alpha_vel;
					_a1 = -alpha_vel;
					_b0 = (1.00 + alpha_vel) / 2.0;
					_b1 = -(1.00 + alpha_vel) / 2.0;
					setName(name);
					_isParamsSet = true;
				}
				else
				{
					_isParamsSet = false;
				}
			}
		}
	}
}
