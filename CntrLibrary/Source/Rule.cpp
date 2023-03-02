#include "Rule.h"

#include <iostream>

namespace CntrlLibrary
{
	Rule::Rule()
	{
	}

	Rule::Rule(const std::string& name, const std::string& definition)
		:_name(name), _definition(definition)
	{
	}

	Rule::~Rule()
	{
	}

	void Rule::addHedge(const std::string& inputName, std::unique_ptr<FuzzyHedge> hedge)
	{
		if (nullptr != hedge)
		{
			_hedges[inputName].push_back(std::move(hedge));;
		}
	}

	void Rule::addInput(FuzzyInput* input)
	{
		if (nullptr != input)
		{
			_ptrInputs.push_back(input);
		}
	}

	void Rule::addOutput(FuzzyOutput* output)
	{
		if (nullptr != output)
		{
			_ptrOutput = output;
		}
	}

	const std::string Rule::getName()
	{
		return _name;
	}

	//returns result of all operationen between inputs
	std::double_t Rule::process()
	{
		std::double_t uresult = 0.00;

		std::vector<std::double_t> results;
		auto iterTerm = _inputTerms.begin();
		std::double_t value;
		//process membership and hedges for each input variable
		for (auto& iterInputs : _ptrInputs)
		{
			std::string input = iterInputs->getName();
			/*find fuzzy Set*/
			FuzzySet* pFuzzySet = iterInputs->getFuzzySet(*iterTerm);
			//get membership value

			value = iterInputs->getValue();
			std::double_t u = pFuzzySet->getMembership(value);

			//std::cout.precision(2);
			//std::cout << "Input " << input.substr(0,1) << "\tFS: " << pFuzzySet->getName().substr(0,2) << "\tVL:" << value << "\tMSF: " << u << std::endl;

			size_t len = _hedges[input].size();
			std::vector<std::unique_ptr<FuzzyHedge>>::iterator vecHedItter = _hedges[input].begin();
			for (size_t i = 0U; i < len; i++)
			{
				FuzzyHedge* pHedge = _hedges[input][len - i - 1U].get();
				u = pHedge->transform(u);
			}
			results.push_back(u);
			iterTerm++;
		}

		if (results.size() > 0U)
		{
			std::double_t ures = results[0];
			//process boolean relations between inputs
			if (results.size() > 1U)
			{
				for (size_t i = 1U; i < results.size(); i++)
				{

					if (_boleanType == BooleanOperation::AndMin)
					{
						//and
						ures = std::fmin(results[i], ures);
					}
					else if (_boleanType == BooleanOperation::AndProduct)
					{
						//and
						ures = results[i] * ures;
					}
					else if (_boleanType == BooleanOperation::OrMax)
					{
						//or
						ures = std::fmax(results[i], ures);
					}
					else if (_boleanType == BooleanOperation::OrProbor)
					{
						//probor
						ures = results[i] + ures - (results[i] * ures);
					}
					else
					{
						throw new std::exception("Not existing boolean operation.");
					}
				}
			}
			//
			uresult = ures;
		}
		else
		{
			uresult = 0.00;
		}
		return uresult;
	}

	void Rule::addInputTerm(const std::string& term)
	{
		_inputTerms.push_back(term);
	}

	void Rule::compile()
	{
		if (_controllerType == FuzzyControllerType::Sugeno)
		{
			FuzzySet* ptrfs = _ptrOutput->getFuzzySet(_outputTerm);
			if (ptrfs != nullptr)
			{


				if( (ptrfs->getMSFType() != FuzzyMembershipFunctionType::LinearSugeno) && (ptrfs->getMSFType() != FuzzyMembershipFunctionType::SingletonSugeno) )
				{
					throw new std::exception("Takagi-Sugeno-Kang fuzzy inference must use singleton output membership functions!");
				}
			}
						
		}
	}

	
	std::double_t Rule::getInputValue(const std::uint32_t i)
	{
		if (_ptrInputs.size() > i)
		{
			return _ptrInputs[i]->getValue();
		}
		else
		{
			throw new std::exception("Out of range!");
		}
		return 0.0;
	}
}