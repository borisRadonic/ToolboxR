/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include "FuzzyControllerMamdani.h"
#include "FuzzyTypes.h"

namespace CntrlLibrary
{
	FuzzyControllerMamdani::FuzzyControllerMamdani() :FuzzyController()
	{
	}

	FuzzyControllerMamdani::FuzzyControllerMamdani(const std::string& name) : FuzzyController(FuzzyControllerType::Mamdani, name)
	{
	}

	FuzzyControllerMamdani::~FuzzyControllerMamdani()
	{
	}

	void FuzzyControllerMamdani::addRule(BooleanOperation booleanType, const std::string& name, const std::string& description, const std::vector<std::string>& inVariables, const std::vector<std::vector<std::string>>& hedges, const std::vector<std::string>& inLingValues, const std::string& outVariable, const std::string& outLingValue, const std::double_t& weight)
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

		rule->setWeight(weight);

		_ptrRules->setFuzzyController(this);

		_ptrRules->AddRule(std::move(rule));
	}

	void FuzzyControllerMamdani::process()
	{
		FuzzyController::process();
	}

	void FuzzyControllerMamdani::compile()
	{
		return FuzzyController::compile();
	}
}