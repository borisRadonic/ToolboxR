#include "Rules.h"
#include "FuzzyController.h"

Rules::Rules()
{
}

Rules::~Rules()
{
}

void Rules::setFuzzyController(FuzzyController* ptrController)
{
	_ptrController = ptrController;
}

void Rules::AddRule(std::unique_ptr<Rule> rule)
{
	if (nullptr != rule)
	{
		std::string name = rule->getName();
		std::string outTerm = rule->getOutputTerm();

		if (_rules.count(name) > 0U)
		{
			throw new std::exception("Already defined rule!");
		}
		_rules[name] = std::move( rule ); //rule will be set to nullptr
	}
}

std::size_t Rules::getNumberOfRules() const
{
	return _rules.size();
}

std::double_t Rules::process()
{	
	return 0.0;
}

void Rules::compile()
{
	if( false == _compiled )
	{
		if (_ptrController == nullptr)
		{
			throw new std::exception("_ptrController == nullptr");
		}
		for (const auto&[key, rule] : _rules)
		{
			rule->compile();
		}				

		for (const auto&[key1, rule1] : _rules)
		{
			/*check if is already on list*/
			bool onList = false;
			for (const auto&r : _rulesOrderByOutTerm)
			{
				if (key1 == r->getName())
				{
					onList = true;
				}				
			}
			if (false == onList)
			{
				_rulesOrderByOutTerm.push_back(rule1.get());
			}
			for (const auto&[key2, rule2] : _rules)
			{
				if (key1 != key2)
				{
					if (rule1->getOutputTerm() == rule2->getOutputTerm())
					{
						onList = false;
						for (const auto&r : _rulesOrderByOutTerm)
						{
							if (r->getName() == rule2->getName())
							{
								onList = true;
							}
						}
						if (false == onList)
						{
							_rulesOrderByOutTerm.push_back(rule2.get());
						}
					}
				}
			}
		}
		for (const auto&[key, rule] : _rules)
		{
			_vecDegOfActOrderByOutTerm.push_back(0.0);
		}
		_compiled = true;
	}
}

Rule* Rules::getRule(size_t i)
{
	auto it = _rules.begin();
	std::advance(it, i);
	return _rules[it->first].get();
}

/////////////////////////////////////////////////////////////////////////////

MamdaniRules::MamdaniRules():Rules()
{
}

MamdaniRules::~MamdaniRules()
{
}

std::double_t MamdaniRules::process()
{
	if (false == _compiled)
	{
		throw new std::exception("Not compiled!");
	}

	std::vector<std::double_t> vecDegOfAcivationUnique;
	std::vector<FuzzyOutput*>	vecOutputsUniqueRules;
	std::vector<std::string>	vecOutputsUniqueTerms;
	
	
	//Claculate the degrees of activation for each rule
	std::uint32_t count = 0U;
	for (const auto&[key2, rule] : _rules)
	{
		_vecDegOfActOrderByOutTerm[count] = rule->getWeight() * rule->process();
		count++;
	}
		
	// Claculate the degrees of activation for the output term (logic OR between the rules with same output term)
	auto iterRules = _rulesOrderByOutTerm.begin();
	std::string outTerm;
	std::double_t valGroup = 0.00;
	count = 0U;
	while( iterRules != _rulesOrderByOutTerm.end() )
	{
		Rule* ptrRule = *iterRules;
		outTerm = ptrRule->getOutputTerm();
		valGroup = 0.00;
		while ((iterRules != _rulesOrderByOutTerm.end()) && (*iterRules)->getOutputTerm() == outTerm)
		{
			valGroup += _vecDegOfActOrderByOutTerm[count];
			count++;
			iterRules++;
		}
		vecDegOfAcivationUnique.push_back(valGroup);
		vecOutputsUniqueRules.push_back(ptrRule->getFuzzyOutput());
		vecOutputsUniqueTerms.push_back(outTerm);
	}	   	 
	
	std::uint32_t resolution = _ptrController->getResolution();
	
	std::double_t min = 0.00;
	std::double_t max = 0.00;
			
	count = 0U;

	std::map<std::string,std::vector<std::double_t>> vecNetOfNets;
	for (const auto& fop: vecOutputsUniqueRules)
	{				
		min = fop->getMinimum();
		max = fop->getMaximum();

		std::double_t len = max - min;
		std::double_t step = len / ((std::double_t) resolution);
		std::double_t x = min;
		std::string name = vecOutputsUniqueTerms[count];
		FuzzySet* fsp = fop->getFuzzySet( name );

		std::vector<std::double_t> vecNetwork;
		std::uint32_t i = 0;
		for (i = 0; i < resolution; i++)
		{
			vecNetwork.push_back(0.0);
		}

		//implication process
		for (std::uint32_t i = 0; i < resolution; i++)
		{						
			//implication		
			if (_ptrController->getFuzzyImplicationMethod() == FuzzyImplicationMethod::Min)
			{
				//Sugeno systems always use the sum aggregation method.
				vecNetwork[i] = std::fmin(vecDegOfAcivationUnique[count], fsp->getMembership(x) );
			}
			else if (_ptrController->getFuzzyImplicationMethod() == FuzzyImplicationMethod::Prod)
			{
				vecNetwork[i] = vecDegOfAcivationUnique[count] * fsp->getMembership(x);
			}					
			x += step;
		}
		vecNetOfNets[name] = vecNetwork;
		count++;
	}
	std::vector<std::double_t> vecOutput;
	std::uint32_t i = 0;
	for (i = 0; i < resolution; i++)
	{
		vecOutput.push_back(0.0);
	}
	//agregation process
	
	count = 0U;
	for (i = 0; i < resolution; i++)
	{		
		for (const auto& net : vecNetOfNets)
		{
			if (_ptrController->getFuzzyAggregationMethod() == FuzzyAggregationMethod::Maximum)
			{
				//Sugeno systems always use the sum aggregation method.
				vecOutput[i] = std::fmax( net.second[count], vecOutput[i]);
			}
			else if (_ptrController->getFuzzyAggregationMethod() == FuzzyAggregationMethod::Sum)
			{
				//Sugeno systems always use the sum aggregation method.
				vecOutput[i] = net.second[count] + vecOutput[i];
			}
			else if (_ptrController->getFuzzyAggregationMethod() == FuzzyAggregationMethod::Probor)
			{
				std::double_t a = std::fmax(net.second[count], vecOutput[i]);
				std::double_t b = vecOutput[i];
				vecOutput[i] = a + b - a * b;
			}
		}
		count++;
	}	

	switch (_ptrController->getDefuzzificationMethod())
	{
		case DefuzzificationMethod::Centroid:
		{
			std::double_t len = max - min;
			std::double_t step = len / ((std::double_t) resolution);
			std::double_t x = min;
			std::double_t num = 0.0;
			std::double_t denum = 0.00;

			for (std::uint32_t i = 0; i < resolution; i++)
			{
				num += vecOutput[i] * x;
				denum += vecOutput[i];
				x += step;;
			}
			if (denum > 0.00)
			{
				return (num / denum);
			}
			return 0.00;
		}
		case DefuzzificationMethod::Bisector:
		{
			throw new std::exception("DefuzzificationMethod is not implementes Method!!!");
		}
		case DefuzzificationMethod::MiddleOfMaximum:
		{
			throw new std::exception("DefuzzificationMethod is not implementes Method!!!");
		}
		case DefuzzificationMethod::LargestOfMaximum:
		{
			throw new std::exception("DefuzzificationMethod is not implementes Method!!!");
		}
		case DefuzzificationMethod::SmallestOfMaximum:
		{
			throw new std::exception("DefuzzificationMethod is not implementes Method!!!");
		}
		default:
		{
			throw new std::exception("No  Defuzzification Method!!!");
		}
	}
}

void MamdaniRules::compile()
{
	Rules::compile();
}

SugenoRules::SugenoRules()
{
}

SugenoRules::~SugenoRules()
{
}

std::double_t SugenoRules::process()
{
	std::map<std::string, std::double_t> vecDegOfAcivation;
	std::map<std::string, std::string> mapRulesNames;

	for (const auto&[key, rule] : _rules)
	{
		vecDegOfAcivation[rule->getOutputTerm()] = 0.0;
	}
	// Claculate the degrees of activation for the output term (logic OR between the rules with same output term)
	for (const auto&[key, res] : vecDegOfAcivation)
	{
		for (const auto&[key2, rule] : _rules)
		{
			if (key == rule->getOutputTerm())
			{
				vecDegOfAcivation[key] = rule->getWeight() * ::fmax(res, rule->process());
				mapRulesNames[key] = rule->getName();
			}
		}
	}
	//Takagi - Sugeno - Kang fuzzy inference uses singleton output membership functions
	std::double_t y0 = 0.00;
	std::double_t y = 0.00;

	std::double_t sigYA = 0.00;
	std::double_t sigA = 0.00;

	for (const auto&[key, res] : vecDegOfAcivation)
	{
		std::string rn = mapRulesNames[key];
		Rule* rule = _rules[rn].get();
		FuzzyOutput* fop = rule->getFuzzyOutput();
				
		sigA += vecDegOfAcivation[key];

		FuzzySet* fsp = fop->getFuzzySet(key); //output singleton Fuzzy set
		if( fsp->getMSFType() == FuzzyMembershipFunctionType::SingletonSugeno)
		{
			std::size_t inMax = rule->getnumberOfInputs();
			y = rule->getoffsetC0();
			for (std::uint32_t i = 0U; i < inMax; i++)
			{
				y += rule->getCoefficient(i) * rule->getInputValue(i);
			}

			sigYA += y * vecDegOfAcivation[key];
		}			
		else if (fsp->getMSFType() == FuzzyMembershipFunctionType::Singleton)
		{
			throw new std::exception("Not supported MembershipFunction in Sugeno system.");
		}
	}
	if (sigA > 0.00)
	{
		return (sigYA / sigA);
	}
	return 0.00;	
}

void SugenoRules::compile()
{
	Rules::compile();
}
