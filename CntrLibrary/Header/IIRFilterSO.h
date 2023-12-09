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

namespace CntrlLibrary
{
	namespace DiscreteTime
	{
		namespace Filters
		{
			class IIRSecondOrderFilter : public Block
			{
			public:

				IIRSecondOrderFilter();

				virtual ~IIRSecondOrderFilter();

				void setParameters(const std::double_t a1, const std::double_t a2, const std::double_t b0, const std::double_t b1, const std::double_t b2, const std::string& name);

				double process(std::double_t u);

				void reset();

				const std::double_t getA1() const
				{
					return _a1;
				}

				const std::double_t getA2() const
				{
					return _a2;
				}

				const std::double_t getB0() const
				{
					return _b0;
				}

				const std::double_t getB1() const
				{
					return _b1;
				}

				const std::double_t getB2() const
				{
					return _b2;
				}

			protected:

				std::shared_ptr<Signal<std::double_t>> _ptrIn;
				std::shared_ptr<Signal<std::double_t>> _ptrOut;

				std::double_t  _a1 = 1.00;       //denominator for z^(-1)
				std::double_t  _a2 = 1.00;       //denominator for z^(-2)

				std::double_t  _b0 = 0.00;       //numerator for z^0
				std::double_t  _b1 = 0.00;       //numerator for z^(-1)
				std::double_t  _b2 = 0.00;       //numerator for z^(-2)

				std::double_t  _x2 = 0.00;       //input at n=-2
				std::double_t  _y2 = 0.00;       //output at n=-2

				std::double_t  _x1 = 0.00;       //input at n=-1
				std::double_t  _y1 = 0.00;       //output at n=-1

				std::double_t  _x0 = 0.00;       //input at n=1
				std::double_t  _y0 = 0.00;       //output at n=1

				bool _isParamsSet = false;
			};
		}
	}
}



