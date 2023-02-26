#include "FuzzyOutput.h"

namespace CntrlLibrary
{
	FuzzyOutput::FuzzyOutput(const std::double_t minimum, const std::double_t maximum, const std::string& name)
		: LinguisticVariable(minimum, maximum, name)
	{
	}

	FuzzyOutput::~FuzzyOutput()
	{
	}
}