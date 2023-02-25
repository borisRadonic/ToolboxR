#include "TrapezoidalIFuzzySetInfL.h"
#include <format>

TrapezoidalIFuzzySetInfL::TrapezoidalIFuzzySetInfL(std::double_t a, std::double_t b, const std::string & name)
:m_a(a), m_b(b), FuzzySet(name)
{
}

FuzzyMembershipFunctionType TrapezoidalIFuzzySetInfL::getMSFType()
{
	return FuzzyMembershipFunctionType::TrapezoidalInfL;
}

std::string TrapezoidalIFuzzySetInfL::getMSFTypeNameFIS()
{
	return std::string("trapmf");
}

std::string TrapezoidalIFuzzySetInfL::getMSFParamExportFISString()
{
	return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + "]");
}

std::double_t TrapezoidalIFuzzySetInfL::getMembership(std::double_t y)
{
	if (y <= m_a )
	{
		return std::double_t(1.0);
	}
	if (y < m_b)
	{
		//y is inside a and b
		return ((m_b - y) / (m_b - m_a));
	}
	//y is outside of the region of universe located on the right side
	return std::double_t(0.0);
}