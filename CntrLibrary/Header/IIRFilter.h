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
			class IIRFilter final : public Block
			{
			public:

				IIRFilter();

				~IIRFilter();

				//Filter coefficients are numerator coefficients and denominator coefficients (vectors must have the same dimensions)
				//
				void setParameters(const std::vector<std::double_t>& numerator, const std::vector<std::double_t>& denominator, const std::string& name = "");

				double process(std::double_t u);

				void reset();

			private:

				std::shared_ptr<Signal<std::double_t>> _ptrIn;
				std::shared_ptr<Signal<std::double_t>> _ptrOut;

				std::vector <std::double_t> _x;
				std::vector <std::double_t> _y;

				bool _isParamsSet = false;

				std::vector<std::double_t> _numerator;
				std::vector<std::double_t> _denominator;
			};
		}
	}
}
