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