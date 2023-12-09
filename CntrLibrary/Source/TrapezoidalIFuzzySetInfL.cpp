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

#include "TrapezoidalIFuzzySetInfL.h"
#include <format>

namespace CntrlLibrary
{
	TrapezoidalIFuzzySetInfL::TrapezoidalIFuzzySetInfL(std::double_t a, std::double_t b, const std::string& name)
		:m_a(a), m_b(b), FuzzySet(name)
	{
	}

	FuzzyMembershipFunctionType TrapezoidalIFuzzySetInfL::getMSFType()
	{
		return FuzzyMembershipFunctionType::TrapezoidalInfL;
	}

	std::string TrapezoidalIFuzzySetInfL::getMSFTypeNameFIS()
	{
		return std::string("trapmf");
	}

	std::string TrapezoidalIFuzzySetInfL::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + "]");
	}

	std::double_t TrapezoidalIFuzzySetInfL::getMembership(std::double_t y)
	{
		if (y <= m_a)
		{
			return std::double_t(1.0);
		}
		if (y < m_b)
		{
			//y is inside a and b
			return ((m_b - y) / (m_b - m_a));
		}
		//y is outside of the region of universe located on the right side
		return std::double_t(0.0);
	}
}