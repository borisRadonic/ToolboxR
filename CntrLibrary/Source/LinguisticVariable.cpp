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
	if (_lingvisticValues.count(name) > 0U)
	{
		return _lingvisticValues[name].get();
	}
	return nullptr;
}

FuzzySet* LinguisticVariable::getFuzzySet(const std::uint32_t index)
{
	if (_lingvisticValues.size() > index)
	{
		auto it = _lingvisticValues.begin();
		std::advance(it, index);
		return it->second.get();		
	}
	return nullptr;
}

void LinguisticVariable::addFuzzySet(std::unique_ptr<FuzzySet> set)
{
	if (nullptr != set)
	{
		std::string name = set->getName();
		if (_lingvisticValues.count(name) > 0U)
		{
			throw new std::exception("Already defined rule!");
		}
		_lingvisticValues[name] = std::move(set); //rule will be set to nullptr
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
