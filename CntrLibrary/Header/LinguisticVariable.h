#pragma once
#include <string>
#include <map>
#include <memory>

#include "FuzzySet.h"

class LinguisticVariable
{
public:

	LinguisticVariable() = delete;

	LinguisticVariable( const std::double_t minimum, const std::double_t maximum, const std::string& name);

	LinguisticVariable(const LinguisticVariable&) = delete;

	LinguisticVariable& operator=(LinguisticVariable&) = delete;

	virtual ~LinguisticVariable();

	const std::string getName()
	{
		return _variableName;
	}

	FuzzySet* getFuzzySet(const std::string& name);

	void addFuzzySet(std::unique_ptr<FuzzySet> set);

	void setValue(const std::double_t value);

	std::double_t getValue();

	std::double_t getMinimum() const
	{
		return _minimum;
	}
	
	std::double_t getMaximum() const
	{
		return _maximum;
	}

protected:

	using MapRLingvisticValuesPtr = std::map<std::string, std::unique_ptr<FuzzySet>>;

	MapRLingvisticValuesPtr _lingvisticValues;

	std::string _variableName;

	std::double_t _minimum;
	std::double_t _maximum;

	std::double_t _value = 0;
};

