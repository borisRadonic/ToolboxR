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

#include "GaussianFuzzySet.h"
#include <math.h>
#include <format>

namespace CntrlLibrary
{
	GaussianFuzzySet::GaussianFuzzySet(std::double_t midpoint, std::double_t spread, const std::string& name)
		:m_midpoint(midpoint), m_spread(spread), FuzzySet(name)
	{
	}

	FuzzyMembershipFunctionType GaussianFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::Gaussian;
	}

	std::string GaussianFuzzySet::getMSFTypeNameFIS()
	{
		return std::string("gaussmf");
	}

	std::string GaussianFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_spread) + " " + std::format("{}", m_midpoint) + "]");
	}

	std::double_t GaussianFuzzySet::getMembership(std::double_t y)
	{
		return exp((-pow(y - m_midpoint, 2.0)) / (2.0 * m_spread * m_spread));
	}
}