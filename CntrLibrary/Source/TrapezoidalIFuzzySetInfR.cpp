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

#include "TrapezoidalIFuzzySetInfR.h"
#include <format>

namespace CntrlLibrary
{
	TrapezoidalIFuzzySetInfR::TrapezoidalIFuzzySetInfR(std::double_t a, std::double_t b, const std::string& name)
		:m_a(a), m_b(b), FuzzySet(name)
	{
	}

	FuzzyMembershipFunctionType TrapezoidalIFuzzySetInfR::getMSFType()
	{
		return FuzzyMembershipFunctionType::TrapezoidalInfR;
	}

	std::string TrapezoidalIFuzzySetInfR::getMSFTypeNameFIS()
	{
		return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + "]");
	}

	std::string TrapezoidalIFuzzySetInfR::getMSFParamExportFISString()
	{
		return std::string();
	}

	std::double_t TrapezoidalIFuzzySetInfR::getMembership(std::double_t y)
	{
		if (y <= m_a)
		{
			return std::double_t(0.0);
		}
		if (y >= m_b)
		{
			return std::double_t(1.0);
		}
		//y is inside a and b
		return ((y - m_a) / (m_b - m_a));

	}
}