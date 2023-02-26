#include "SShapedFuzzySet.h"
#include <format>

namespace CntrlLibrary
{
	SShapedFuzzySet::SShapedFuzzySet(std::double_t a, std::double_t b, const std::string& name)
		:m_a(a), m_b(b), FuzzySet(name)
	{
	}

	FuzzyMembershipFunctionType SShapedFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::SShaped;
	}

	std::string SShapedFuzzySet::getMSFTypeNameFIS()
	{
		return std::string("smf");
	}

	std::string SShapedFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + "]");
	}

	std::double_t SShapedFuzzySet::getMembership(std::double_t y)
	{
		if (y <= m_a)
		{
			return std::double_t(0.0);
		}

		if ((y >= m_a) && (y <= ((m_a + m_b) / 2.0)))
		{
			return(2.0 * pow(((y - m_a) / (m_b - m_a)), 2.0));
		}

		if ((y >= ((m_a + m_b) / 2.0)) && (y <= m_b))
		{
			return(1.0 - 2.0 * pow(((y - m_b) / (m_b - m_a)), 2.0));
		}

		//  y >= m_b
		if (y >= m_b)
		{
			return std::double_t(1.0);
		}
		return std::double_t(1.0);
	}
}