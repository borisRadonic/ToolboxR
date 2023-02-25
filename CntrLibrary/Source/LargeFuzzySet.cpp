#include "LargeFuzzySet.h"
#include <format>

LargeFuzzySet::LargeFuzzySet(std::double_t midpoint, std::double_t spread, const std::string & name)
	:m_midpoint(midpoint), m_spread(spread), FuzzySet(name)
{
}

FuzzyMembershipFunctionType LargeFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::FuzzySmall;
}

std::string LargeFuzzySet::getMSFTypeNameFIS()
{
	return std::string("large");
}

std::string LargeFuzzySet::getMSFParamExportFISString()
{
	return ("[" + std::format("{}", m_midpoint) + " " + std::format("{}", m_spread) + "]");
}

std::double_t LargeFuzzySet::getMembership(std::double_t y)
{
	if (abs(m_midpoint) > MIN_DOUBLE_DIFF)
	{
		return (1.0 / (1 + pow((y / m_midpoint), -m_spread)));
	}
	return std::double_t(0.0);
}