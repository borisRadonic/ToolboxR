#include "BellShapedFuzzySet.h"
#include <math.h>


BellShapedFuzzySet::BellShapedFuzzySet(std::double_t midpoint, std::double_t width, std::double_t slope, const std::string& name )
:m_midpoint(midpoint), m_a(width/2.0), m_b(width*slope), FuzzySet(name)
{
}

FuzzyMembershipFunctionType BellShapedFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::BellShaped;
}

std::double_t BellShapedFuzzySet::getMembership(std::double_t y)
{
	if (m_a <= 0.0 )
	{
		return std::double_t(0.0);
	}
	return (1.0 / ( 1.0 + pow( abs( (y - m_midpoint) / m_a), 2.0*m_b)));
}