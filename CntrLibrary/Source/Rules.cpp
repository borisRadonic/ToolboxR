#include "Rules.h"
#include "FuzzyController.h"
#include "SingletonFuzzySet.h"

namespace CntrlLibrary
{
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
			_rules[name] = std::move(rule); //rule will be set to nullptr
		}
	}

	std::size_t Rules::getNumberOfRules() const
	{
		return _rules.size();
	}

	std::vector<std::string> Rules::getNames()
	{
		std::vector<std::string> vec;
		for (const auto& [key, rule] : _rules)
		{
			vec.push_back(key);
		}
		return vec;
	}

	std::double_t Rules::process()
	{
		return 0.0;
	}

	void Rules::compile()
	{
		if (false == _compiled)
		{
			if (_ptrController == nullptr)
			{
				throw new std::exception("_ptrController == nullptr");
			}
			for (const auto& [key, rule] : _rules)
			{
				rule->compile();
			}

			for (const auto& [key1, rule1] : _rules)
			{
				/*check if is already on list*/
				bool onList = false;
				for (const auto& r : _rulesOrderByOutTerm)
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
				for (const auto& [key2, rule2] : _rules)
				{
					if (key1 != key2)
					{
						if (rule1->getOutputTerm() == rule2->getOutputTerm())
						{
							onList = false;
							for (const auto& r : _rulesOrderByOutTerm)
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
			for (const auto& [key, rule] : _rules)
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

	Rule* Rules::getRule(std::string name)
	{
		if (_rules[name] != nullptr)
		{
			return _rules[name].get();
		}
		return nullptr;
	}

	/////////////////////////////////////////////////////////////////////////////

	MamdaniRules::MamdaniRules() :Rules()
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
		for (const auto& [key2, rule] : _rules)
		{
			_vecDegOfActOrderByOutTerm[count] = rule->getWeight() * rule->process();
			count++;
		}

		// Claculate the degrees of activation for the output term (logic OR between the rules with same output term)
		auto iterRules = _rulesOrderByOutTerm.begin();
		std::string outTerm;
		std::double_t valGroup = 0.00;
		count = 0U;
		while (iterRules != _rulesOrderByOutTerm.end())
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

		std::double_t mmin = 0.00;
		std::double_t mmax = 0.00;

		count = 0U;

		std::map<std::string, std::vector<std::double_t>> vecNetOfNets;
		for (const auto& fop : vecOutputsUniqueRules)
		{
			min = fop->getMinimum();
			max = fop->getMaximum();

			if (min < mmin)
			{
				mmin = min;
			}
			if (max > mmax)
			{
				mmax = max;
			}
			std::double_t len = max - min;
			std::double_t step = len / ((std::double_t)resolution);
			std::double_t x = min;
			std::string name = vecOutputsUniqueTerms[count];
			FuzzySet* fsp = fop->getFuzzySet(name);

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
					vecNetwork[i] = std::fmin(vecDegOfAcivationUnique[count], fsp->getMembership(x));
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
					vecOutput[i] = std::fmax(net.second[count], vecOutput[i]);
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
			std::double_t len = mmax - mmin;
			std::double_t step = len / ((std::double_t)resolution);
			std::double_t x = mmin;
			std::double_t num = 0.0;
			std::double_t denum = 0.00;

			for (std::uint32_t i = 0; i < resolution; i++)
			{
				num += vecOutput[i] * x;
				denum += vecOutput[i];
				x += step;
			}
			if (denum > 0.00)
			{
				return (num / denum);
			}
			return 0.00;
		}
		case DefuzzificationMethod::Bisector:
		{
			std::double_t len = mmax - mmin;
			std::double_t step = len / ((std::double_t)resolution);
			std::double_t x = mmin;
			std::double_t y = mmax;
			std::double_t left = 0.00;
			std::double_t right = 0.00;

			for (std::uint32_t i = 0; i < resolution; i++)
			{
				if (left > right)
				{
					right += vecOutput[resolution - 1 - i] * y;
					y = y - step;

				}
				else
				{					
					x = x + step;
					left += vecOutput[i] * x;
				}

				if ( (abs( left - right)<0.0000001) && (left!= 0.00) && (right != 0.00))
				{
					//end
					return x;
					break;
				}
			}
			return ((x+y)/2.00);

		}
		case DefuzzificationMethod::MiddleOfMaximum:
		{
			std::double_t len = mmax - mmin;
			std::double_t step = len / ((std::double_t)resolution);
			std::double_t x = mmin;
			std::double_t maxx = 0.0;
			std::double_t minx = 0.00;
			std::double_t lastx = 0.00;
			std::double_t mid = 0.00;

			std::uint32_t f = 0;
			for (std::uint32_t i = 0; i < resolution; i++)
			{
				if (vecOutput[i] > (maxx + 0.000000001))
				{
					maxx = vecOutput[i];
					minx = x;
					f = i;
				}
				x += step;;
			}

			x = minx;
			i = f;
			while (vecOutput[i] == maxx)
			{
				i++;
			}
			return (minx + lastx) / 2.0;
		}
		case DefuzzificationMethod::LargestOfMaximum:
		{
			std::double_t len = mmax - mmin;
			std::double_t step = len / ((std::double_t)resolution);
			std::double_t x = mmin;
			std::double_t maxx = 0.0;
			std::double_t lastx = 0.00;

			for (std::uint32_t i = 0; i < resolution; i++)
			{
				if (vecOutput[i] >= maxx)
				{
					maxx = vecOutput[i];
					lastx = x;
				}
				x += step;;
			}
			return lastx;
		}
		case DefuzzificationMethod::SmallestOfMaximum:
		{
			std::double_t len = mmax - mmin;
			std::double_t step = len / ((std::double_t)resolution);
			std::double_t x = mmin;
			std::double_t maxx = 0.0;
			std::double_t minx = 0.00;

			for (std::uint32_t i = 0; i < resolution; i++)
			{
				if (vecOutput[i] > (maxx + 0.000000001))
				{
					maxx = vecOutput[i];
					minx = x;
				}
				x += step;;
			}
			return minx;
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
		std::double_t Wi = 0.00;
		std::double_t Zi = 0.00;
		std::double_t prodWiZi = 0.00;
		std::double_t prodWi = 0.00;

		for (const auto& [key, rule] : _rules)
		{
			
			Wi = rule->getWeight() * rule->process();

			std::string oterm = rule->getOutputTerm();
			FuzzyOutput* fo = rule->getFuzzyOutput();
			if (fo != nullptr)
			{
				FuzzySet* fs = fo->getFuzzySet(oterm);;
				if (fs != nullptr)
				{
					if (fs->getMSFType() == FuzzyMembershipFunctionType::LinearSugeno)
					{
						if (rule->getnumberOfInputs() < 2U)
						{
							throw new std::exception("The numer of inputs in LinearSugeno must be >= 2");
						}
						else
						{
							LinearSugenoFuzzySet* pss = (LinearSugenoFuzzySet*) fs;
							if (pss != nullptr)
							{
								Zi = pss->get(rule->getInputValue(0), rule->getInputValue(1));
							}
						}
					}
					else if (fs->getMSFType() == FuzzyMembershipFunctionType::SingletonSugeno)
					{
						//zero order sugeno system -> Zi is constant
						SingletonSugenoFuzzySet* pss = (SingletonSugenoFuzzySet*)fs;
						if (pss != nullptr)
						{
							Zi = pss->getConstant();
						}
					}
					else
					{
						throw new std::exception("Not supported MembershipFunction in Sugeno system.");
					}
				}
			}
			prodWiZi += (Wi * Zi);
			prodWi += Wi;
			
		}
		return(prodWiZi / prodWi);
	}

	void SugenoRules::compile()
	{

		if (false == _compiled)
		{
			if (_ptrController == nullptr)
			{
				throw new std::exception("_ptrController == nullptr");
			}
			for (const auto& [key, rule] : _rules)
			{
				rule->compile();
			}

			//check the structure of the rules
			for (const auto& [key, rule] : _rules)
			{
				if (key == rule->getOutputTerm())
				{
					std::string oterm = rule->getOutputTerm();
					FuzzyOutput* fo = rule->getFuzzyOutput();
					if (fo != nullptr)
					{
						FuzzySet* fs = fo->getFuzzySet("oterm");;
						if (fs == nullptr)
						{
							throw new std::exception("fs == nullptr");
						}
						if (fs != nullptr)
						{
							if (fs->getMSFType() == FuzzyMembershipFunctionType::LinearSugeno)
							{
								if (rule->getnumberOfInputs() < 2U)
								{
									throw new std::exception("The numer of inputs in LinearSugeno must be >= 2");
								}
								continue;
							}
							else if (fs->getMSFType() == FuzzyMembershipFunctionType::SingletonSugeno)
							{
								continue;
							}
							else
							{
								throw new std::exception("Not supported MembershipFunction in Sugeno system.");
							}
						}
					}
				}
			}
		}
	}
}
