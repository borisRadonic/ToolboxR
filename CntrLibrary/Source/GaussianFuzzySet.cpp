#include "GaussianFuzzySet.h"
#include <math.h>
#include <format>


GaussianFuzzySet::GaussianFuzzySet(std::double_t midpoint, std::double_t spread, const std::string & name)
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
	return ("[" + std::format("{}", m_spread) + " " + std::to_string(m_midpoint) + "]");
}

std::double_t GaussianFuzzySet::getMembership(std::double_t y)
{
	return exp(  ( -pow(y - m_midpoint, 2.0) )/(2.0* m_spread* m_spread));
}
