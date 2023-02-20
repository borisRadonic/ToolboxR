#pragma once
#include "FuzzyController.h"

class FuzzyControllerSugeno final : public FuzzyController
{
public:

	FuzzyControllerSugeno();

	FuzzyControllerSugeno(const std::string& name);

	virtual ~FuzzyControllerSugeno();
		
	virtual void addRule(	BooleanOperation booleanType,
							const std::string & name,
							const std::string & description, const std::vector<std::string>& inVariables,
							const std::vector<std::vector<std::string>>& hedges,
							const std::vector<std::string>& inLingValues,
							const std::string & outVariable,
							const std::string & outLingValue,
							const std::vector<double_t>& thenCoefsC, //THEN C1...Cr for Sugeno system
							const std::double_t & offset, //THEN C0 for Sugeno system  y = c0 + C1*X1 + c2*X2...Cn*Xn
							const std::double_t & weight) override; //Optional weight for Mamdani system

	virtual void process() override;

	virtual void compile() override;

};