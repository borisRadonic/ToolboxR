#include <string>
#include <map>
#include <memory>
#include "LinguisticVariable.h"

#pragma once
class FuzzyInput final : public LinguisticVariable
{
public:
	FuzzyInput() = delete;

	FuzzyInput(std::double_t minimum, std::double_t maximum, const std::string& name = "" );

	 ~FuzzyInput();
};

