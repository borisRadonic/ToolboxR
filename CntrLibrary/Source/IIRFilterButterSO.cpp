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

#include "IIRFilterButterSO.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{

			/// <summary>
			/// Butterworth LP Filter
			/// </summary>
			/// 
			ButterworthLowPassII::ButterworthLowPassII()
			{
			}

			ButterworthLowPassII::~ButterworthLowPassII()
			{
			}

			void ButterworthLowPassII::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{
					std::double_t f_c = omega_c / (2 * M_PI);
					std::double_t f_s = 1.00 / ts;

					std::double_t omega_norm =  f_c / (f_s/2.00);

					std::double_t omega_p = 1.00/  tan( M_PI * f_c/ f_s );
					
					std::double_t  omega_p_squ = omega_p * omega_p;
					_b0 = 1.00 / (1.00 + M_SQRT2 * omega_p + omega_p_squ);
					_b1 = 2.00 * _b0;
					_b2 = _b0;

					
					_a1 = -(2.00 * (omega_p_squ - 1.00)) * _b0;
					_a2 =  (1.00 - M_SQRT2 * omega_p + omega_p_squ) * _b0;

					setName(name);
					_isParamsSet = true;
				}
				else
				{
					_isParamsSet = false;
				}
			}

			/// <summary>
			/// Butterworth High-pass Filter
			/// </summary>
			ButterworthHighPassII::ButterworthHighPassII()
			{
			}

			ButterworthHighPassII::~ButterworthHighPassII()
			{
			}

			void ButterworthHighPassII::setCutoffFrequency(const std::double_t omega_c, const std::double_t ts, const std::string& name)
			{
				_omega_c = omega_c;
				_ts = ts;
				if ((ts > 0.00) && (omega_c > 0.00))
				{

					std::double_t f_c = omega_c / (2 * M_PI);
					std::double_t f_s = 1.00 / ts;
					std::double_t omega_p = 1.00 / tan(M_PI * f_c / f_s);

					std::double_t  omega_p_squ = omega_p * omega_p;
					_b0 = 1.00 / (1.00 + M_SQRT2 * omega_p + omega_p_squ);
					_b1 = 2.00 * _b0;
					_b2 = _b0;

					_a1 = -(2.00 * (omega_p_squ - 1.00)) * _b0;
					_a2 = (1.00 - M_SQRT2 * omega_p + omega_p_squ) * _b0;

					_b0 = _b0 * omega_p_squ;
					_b1 = -_b1 * omega_p_squ;
					_b2 = _b2 * omega_p_squ;

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