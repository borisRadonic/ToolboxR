#include "FuzzyControllerSugeno.h"

namespace CntrlLibrary
{
	FuzzyControllerSugeno::FuzzyControllerSugeno() :FuzzyController()
	{
	}

	FuzzyControllerSugeno::FuzzyControllerSugeno(const std::string& name) : FuzzyController(FuzzyControllerType::Sugeno, name)
	{
	}

	FuzzyControllerSugeno::~FuzzyControllerSugeno()
	{
	}

	void FuzzyControllerSugeno::addRule(BooleanOperation booleanType, const std::string& name, const std::string& description, const std::vector<std::string>& inVariables, const std::vector<std::vector<std::string>>& hedges, const std::vector<std::string>& inLingValues, const std::string& outVariable, const std::string& outLingValue, const std::double_t& weigh)
	{
		if (inVariables.size() != inLingValues.size())
		{
			throw new std::exception("Rule syntax error. Number of input variables != number of values");
		}

		if ((outVariable.size() == 0U) || (outLingValue.size() == 0U))
		{
			throw new std::exception("Rule syntax error: Output");
		}

		if (hedges.size() != inVariables.size())
		{
			throw new std::exception("Rule syntax error. Number of input variables != number of hedges");
		}
		
		std::unique_ptr<Rule> rule = std::make_unique<Rule>(name, description);

		bool bFound = false;
		for (auto& iterInVars : inVariables)
		{
			bFound = false;
			for (auto& in : _inputs)
			{
				if (in.get()->getName() == iterInVars)
				{
					bFound = true;
				}
			}
			if (false == bFound)
			{
				throw new std::exception("Rule error. Input variable does not exists.");
			}
		}

		if (_output->getName() != outVariable)
		{
			throw new std::exception("Rule error. Output variable does not exists.");
		}

		auto iterInput = _inputs.begin();
		auto iterInVars = inVariables.begin();
		for (auto& iterLingVars : inLingValues)
		{
			FuzzyInput* input = getInput(*iterInVars);
			if (input == nullptr)
			{
				throw new std::exception("Rule error: unexisting input");
			}
			FuzzySet* set = input->getFuzzySet(iterLingVars);
			if (set == nullptr)
			{
				throw new std::exception("Rule error: unexisting term");
			}
			rule->addInput(input);
			rule->addInputTerm(iterLingVars);
			iterInput++;
			iterInVars++;
		}


		iterInVars = inVariables.begin();
		for (auto& hedge : hedges)
		{
			if (hedge.size() == 0)
			{
				throw new std::exception("Rule error: empty hedge.");
			}
			for (auto& hOneIter : hedge)
			{
				FuzzyHedge::FuzzyHedgeType type = FuzzyHedge::getFuzzyHedgeTypeFromString(hOneIter);
				if (type == FuzzyHedge::FuzzyHedgeType::NotExisting)
				{
					throw new std::exception("Rule error: not existing hedge.");
				}
				rule->addHedge(*iterInVars, std::move(FuzzyHedge::create(type)));
			}
			iterInVars++;
		}

		if (_output == nullptr)
		{
			throw new std::exception("Rule error: unexisting output");
		}
		FuzzySet* set = _output->getFuzzySet(outLingValue);
		if (set == nullptr)
		{
			throw new std::exception("Rule error: unexisting term");
		}

		rule->addOutput(_output.get());

		rule->setOutputTerm(outLingValue);

		rule->setBooleanType(booleanType);

		rule->setControllerType(_controllerType);

		rule->setWeight(weigh);

		_ptrRules->setFuzzyController(this);

		_ptrRules->AddRule(std::move(rule));
	}

	void FuzzyControllerSugeno::process()
	{
		FuzzyController::process();
	}

	void FuzzyControllerSugeno::compile()
	{
		_agregationmethod = FuzzyAggregationMethod::Sum;
		return FuzzyController::compile();
	}
}