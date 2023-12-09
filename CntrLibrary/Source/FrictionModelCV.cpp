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

#include "FrictionModelCV.h"
#include <math.h>

namespace CntrlLibrary
{
	namespace Models
	{
		FrictionModelCSV::FrictionModelCSV()
		{
		}
		FrictionModelCSV::~FrictionModelCSV()
		{
		}

		void FrictionModelCSV::setParameters(std::double_t ts, std::double_t b, double_t c0, std::double_t J)
		{
			_Ts = ts;
			_B = b;
			_C0 = c0;
			_J = J;
			_isParamsSet = true;
		}

		void FrictionModelCSV::reset()
		{
			_w = 0.00;
			_a = 0.00;
			_T = 0.00;
		}


		void FrictionModelCSV::setInputs(std::double_t w, std::double_t ti, std::double_t a)
		{
			_w = w;
			_Ti = ti;
			_a = a;
		}

		void FrictionModelCSV::process()
		{
			if (_isParamsSet)
			{
				//Tf = Ts +Tc + Tv
				//Ts - static friction
				//Tc - Coulomb or kinetic friction
				//Tv - viscous friction
				std::double_t  tc = 0 - 00;
				std::double_t  tv = _B * _w;
				std::double_t  ts = 0.00;
				std::double_t  signW = 1.00;
				std::double_t  signT = 1.00;

				if (std::signbit(_w))
				{
					//negative velocity
					signW = -1.00;
				}

				if (std::signbit(_Ti))
				{
					//negative input torque
					signT = -1.00;
				}

				if (std::abs(_a) < std::numeric_limits<std::double_t>::min())
				{
					//acceleration is != 0				

					if (abs(_Ti - (_J * _a)) < _C0)
					{
						ts = _Ti;
					}
					else
					{
						ts = _C0 * signW;
					}
				}
				else
				{
					if (abs(_Ti) < _C0)
					{
						ts = _Ti;
					}
					else
					{
						ts = _C0 * signT;
					}
				}

				if (std::abs(_w) >= std::numeric_limits<std::double_t>::min())
				{
					{
						ts = 0.00; //velocity > 0.00
						tc = _C0 * signW;
					}
				}
				_T = ts + tc + tv;
			}
		}
	}
}