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

#include "TriangularFuzzySet.h"
#include <format>

namespace CntrlLibrary
{
	TriangularFuzzySet::TriangularFuzzySet(std::double_t a, std::double_t b, std::double_t c, const std::string& name)
		:m_a(a), m_b(b), m_c(c), FuzzySet(name)
	{
	}

	FuzzyMembershipFunctionType TriangularFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::Triangular;
	}

	std::string TriangularFuzzySet::getMSFTypeNameFIS()
	{
		return std::string("trimf");
	}

	std::string TriangularFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + " " + std::format("{}", m_c) + "]");
	}

	std::double_t TriangularFuzzySet::getMembership(std::double_t y)
	{
		if ((y <= m_a) || (y >= m_c))
		{
			return std::double_t(0.0);
		}

		//y is inside a and c
		if (y <= m_b)
		{
			//y is inside a and b
			if ((m_b - m_a) > 0.0)
			{
				//y is inside a and b
				return((y - m_a) / (m_b - m_a));
			}
			else
			{
				//something is wrong
				if (abs(m_b - m_a) <= MIN_DOUBLE_DIFF)
				{
					//a = b
					return std::double_t(1.0);
				}
				//b>a !!!!!
				return std::double_t(0.0);
			}
		}
		//y is inside b and c
		return ((m_c - y) / (m_c - m_b));
	}
}