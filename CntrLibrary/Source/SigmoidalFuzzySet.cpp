#include "SigmoidalFuzzySet.h"
#include <math.h>
#include <format>

SigmoidalFuzzySet::SigmoidalFuzzySet(std::double_t a, std::double_t c,const std::string& name)
:m_a(a), m_c(c), FuzzySet(name)
{
}

FuzzyMembershipFunctionType SigmoidalFuzzySet::getMSFType()
{
	return FuzzyMembershipFunctionType::Sigmoidal;
}

std::string SigmoidalFuzzySet::getMSFTypeNameFIS()
{
	return std::string("sigmf");
}

std::string SigmoidalFuzzySet::getMSFParamExportFISString()
{
	return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_c) + "]");
}

std::double_t SigmoidalFuzzySet::getMembership(std::double_t y)
{
	if (m_a <= 0.0)
	{
		return std::double_t(0.0);
	}
	return (1.0 / (1.0 + exp(- m_a * (y - m_c) )));
}
