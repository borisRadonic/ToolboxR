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

#include "SingletonFuzzySet.h"
#include <format>

namespace CntrlLibrary
{
	FuzzyMembershipFunctionType SingletonFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::Singleton;
	}

	std::string SingletonFuzzySet::getMSFTypeNameFIS()
	{
		return std::string("singleton");
	}

	std::string SingletonFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_c) + "]");
	}

	std::double_t SingletonFuzzySet::getMembership(std::double_t y)
	{
		std::double_t m = MIN_DOUBLE_DIFF;
		std::double_t max = m_c + m;
		std::double_t min = m_c - m;

		if ((y >= min) && (y <= max))
		{
			return std::double_t(1.0);
		}
		return std::double_t(0.0);;
	}

	FuzzyMembershipFunctionType SingletonSugenoFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::SingletonSugeno;
	}

	std::string SingletonSugenoFuzzySet::getMSFTypeNameFIS()
	{
		return "constant";
	}

	std::string SingletonSugenoFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_c) + "]");
	}

	std::double_t SingletonSugenoFuzzySet::getMembership(std::double_t y)
	{
		//not used for saugeno system
		return std::double_t(1.00);
	}
	FuzzyMembershipFunctionType LinearSugenoFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::LinearSugeno;
	}
	std::string LinearSugenoFuzzySet::getMSFTypeNameFIS()
	{
		return "linear";
	}
	std::string LinearSugenoFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + " " + std::format("{}", m_c) + "]");
	}
	std::double_t LinearSugenoFuzzySet::getMembership(std::double_t y)
	{
		//not used for saugeno system
		return std::double_t(1.00);
	}
}