#include "GaussianFuzzySet.h"
#include <math.h>


GaussianFuzzySet::GaussianFuzzySet(std::double_t midpoint, std::double_t spread, const std::string & name)
:m_midpoint(midpoint), m_spread(spread), FuzzySet(name)
{
}

FuzzyMembershipFunctionType GaussianFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::Gaussian;
}

std::double_t GaussianFuzzySet::getMembership(std::double_t y)
{
	return exp(  ( -pow(y - m_midpoint, 2.0) )/(2.0* m_spread* m_spread));
}
