#include "SingletonFuzzySet.h"

FuzzyMembershipFunctionType SingletonFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::Singleton;
}

std::double_t SingletonFuzzySet::getMembership(std::double_t y)
{
	std::double_t m = MIN_DOUBLE_DIFF;
	std::double_t max = m_c + m;
	std::double_t min = m_c - m;

	if ( (y >= min) && (y <= max) )
	{
		return std::double_t(1.0);
	}
	return std::double_t(0.0);;
}

FuzzyMembershipFunctionType SingletonSugenoFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::SingletonSugeno;
}

std::double_t SingletonSugenoFuzzySet::getMembership(std::double_t y)
{
	return std::double_t(1.00);
}