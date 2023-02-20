#include <string>
#include <map>
#include <memory>

#include "LinguisticVariable.h"

#pragma once
class FuzzyOutput final : public LinguisticVariable
{
public:
	FuzzyOutput() = delete;

	FuzzyOutput(const std::double_t minimum, const std::double_t maximum, const std::string& name = "");

	~FuzzyOutput();


};
