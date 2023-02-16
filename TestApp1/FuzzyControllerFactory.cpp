#include "FuzzyControllerFactory.h"


FuzzyController * FuzzyControllerFactory::createController(const FuzzyControllerType & type, const std::string & name)
{
	switch (type)
	{
		case FuzzyControllerType::Mamdani:
		{
			return new FuzzyControllerMamdani(name);
		}
		case FuzzyControllerType::Sugeno:
		{
			return new FuzzyControllerSugeno(name);
		}
		default:
			throw new std::exception("Unknown FuzzyControllerType.");
	}
	return nullptr;
}
