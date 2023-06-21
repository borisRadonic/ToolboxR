#pragma once
#include "FuzzyControllerMamdani.h"
#include "FuzzyControllerSugeno.h"
#include "FuzzyTypes.h"

namespace CntrlLibrary
{
	class FuzzyControllerFactory
	{
	public:
		static FuzzyController* createController(const FuzzyControllerType& type, const std::string& name);
	};
};
