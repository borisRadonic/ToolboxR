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
#include "FuzzySet.h"

namespace CntrlLibrary
{
	// The generalized bell function depends on three parameters a, b and c as given by : f(x;a,b,midpoint) = 1/ ((1+ abs(x-midpoint))/ (2a)) ^2b)
	// Midpoint determines the centre of the corresponding membership function
	//a is the half width;
	//a and b together control the slopes at the crossover points -> slope = b/2a

	class BellShapedFuzzySet final : public FuzzySet
	{
	public:
		BellShapedFuzzySet() = delete;

		BellShapedFuzzySet(std::double_t midpoint, std::double_t width, std::double_t slope, const std::string& name = "");

		virtual ~BellShapedFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;

		virtual std::double_t getMembership(std::double_t y) final;

	private:
		std::double_t m_midpoint;
		std::double_t m_a;
		std::double_t m_b;
	};
};

