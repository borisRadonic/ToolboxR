#include "LinguisticVariable.h"

LinguisticVariable::LinguisticVariable( const std::double_t minimum, const std::double_t maximum, const std::string & name)
:_minimum(minimum), _maximum(maximum), _variableName(name), _value(0.0)
{
}

LinguisticVariable::~LinguisticVariable()
{
}

FuzzySet* LinguisticVariable::getFuzzySet(const std::string & name)
{
	for (auto& lv : _lingvisticValues)
	{
		if (lv.get()->getName() == name)
		{
			return lv.get();
		}
	}
	return nullptr;
}

void LinguisticVariable::addFuzzySet(std::unique_ptr<FuzzySet> set)
{
	if (nullptr != set)
	{
		std::string name = set->getName();

		for (auto& lv : _lingvisticValues)
		{
			if (lv.get()->getName() == name)
			{
				throw new std::exception("Already defined rule!");
			}
		}
		_lingvisticValues.push_back(std::move(set));
	}
}

void LinguisticVariable::setValue(const std::double_t value)
{
	if (value < HUGE_VAL )
	{
		if ((value >= _minimum) && (value <= _maximum) )
		{
			_value = value;
		}
		else
		{
			throw new std::exception("Input range violation");
		}
	}
	else
	{
		throw new std::exception("double NAN");
	}
}

std::double_t LinguisticVariable::getValue()
{
	return _value;
}

std::vector<std::string> LinguisticVariable::getLingvisticValues()
{
	std::vector<std::string> vec;
	for (auto& lv : _lingvisticValues)
	{
		vec.push_back(lv->getName());
	}
	return vec;
}
