#include "SmallFuzzySet.h"
#include <format>

SmallFuzzySet::SmallFuzzySet(std::double_t midpoint, std::double_t spread, const std::string & name)
:m_midpoint(midpoint), m_spread(spread), FuzzySet(name)
{
}

FuzzyMembershipFunctionType SmallFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::FuzzySmall;
}

std::string SmallFuzzySet::getMSFTypeNameFIS()
{
	return std::string("small");
}

std::string SmallFuzzySet::getMSFParamExportFISString()
{
	return ("[" + std::format("{}", m_midpoint) + " " + std::format("{}", m_spread) + "]");
}

std::double_t SmallFuzzySet::getMembership(std::double_t y)
{
	if (abs(m_midpoint) > MIN_DOUBLE_DIFF)
	{
		return (1.0 / (1 + pow((y / m_midpoint), m_spread)));
	}
	return std::double_t(0.0);
}