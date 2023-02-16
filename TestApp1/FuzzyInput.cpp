#include "FuzzyInput.h"


FuzzyInput::FuzzyInput(std::double_t minimum, std::double_t maximum, const std::string& name)
: LinguisticVariable(minimum, maximum, name)
{
}

FuzzyInput::~FuzzyInput()
{
}
