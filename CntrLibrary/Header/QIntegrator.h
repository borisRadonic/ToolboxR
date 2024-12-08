
/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2024 Boris Radonic

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

#ifndef QINTEGRATOR_H
#define QINTEGRATOR_H

#include <cstdint>
#include <algorithm> // For std::min etc.
#include <cmath>
#include <limits>
#include "FixedPoint.h"


namespace CntrlLibrary
{
	template <typename TYPE_IN_OUT, typename TYPE_KI>
	class QIntegrator
	{
	public:


		explicit QIntegrator(float ki, float min, float max):Ki(ki), outMin(min), outMax(max)
		{			
		}

		~QIntegrator() = default;

		
		void setInitialConditions(TYPE_IN_OUT ic)
		{
			this->ic = ic;
			this->x = ic;
			this->y = ic;
		}

		
		void reset()
		{
			this->ic = TYPE_IN_OUT(0.0f);
		}

		auto process(auto u)
		{
			auto val1 = FixedPointOps::mul(u1, Ki);
			auto val2 = FixedPointOps::add(val1, x);
			FixedPointOps::convert(val2, x);
			FixedPointOps::convert(u, u1);
			return x;
		}

	private:

		TYPE_KI Ki{0.0f};
		TYPE_IN_OUT outMin{}; //Output saturation limit
		TYPE_IN_OUT outMax{}; //Output saturation limit
		bool isUseSaturation{ false };
		TYPE_IN_OUT ic{ 0.0f }; //initial conditions
		TYPE_IN_OUT x{ 0.0f };
		TYPE_IN_OUT u1{ 0.0f };
	};
}
#endif