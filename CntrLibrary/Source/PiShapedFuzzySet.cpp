#include "PiShapedFuzzySet.h"
#include <format>

namespace CntrlLibrary
{
	PiShapedFuzzySet::PiShapedFuzzySet(std::double_t a, std::double_t b, std::double_t c, std::double_t d, const std::string& name)
		:m_a(a), m_b(b), m_c(c), m_d(d), FuzzySet(name)
	{
	}

	FuzzyMembershipFunctionType PiShapedFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::PiShaped;
	}

	std::string PiShapedFuzzySet::getMSFTypeNameFIS()
	{
		return std::string("pimf");
	}

	std::string PiShapedFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + " " + std::format("{}", m_c) + " " + std::format("{}", m_d) + "]");
	}

	std::double_t PiShapedFuzzySet::getMembership(std::double_t y)
	{
		if (y <= m_a || y >= m_d)
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

		if ((y >= m_b) && (y <= m_c))
		{
			return std::double_t(1.0);
		}


		if ((y >= m_c) && (y <= ((m_c + m_d) / 2.0)))
		{
			return(1.0 - 2.0 * pow(((y - m_c) / (m_d - m_c)), 2.0));
		}

		if ((y >= ((m_c + m_d) / 2.0)) && (y <= m_d))
		{
			return(2.0 * pow(((y - m_d) / (m_d - m_c)), 2.0));
		}

		//  y >= m_d
		return std::double_t(1.0);
	}
}