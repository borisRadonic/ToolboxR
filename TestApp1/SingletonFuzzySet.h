#pragma once
#include "FuzzySet.h"

/*
SingletonSugenoFuzzySet Fuzzy set is used only for output and only with Sugeno  system
*/

class SingletonFuzzySet final : public FuzzySet
{
public:
	SingletonFuzzySet() = delete;

	SingletonFuzzySet(std::double_t c, const std::string& name = "") :m_c(c), FuzzySet(name) {}

	virtual ~SingletonFuzzySet(){}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_c;
};


class SingletonSugenoFuzzySet final : public FuzzySet
{
public:
	SingletonSugenoFuzzySet() = delete;

	SingletonSugenoFuzzySet(const std::string& name = "") :FuzzySet(name) {}

	virtual ~SingletonSugenoFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::double_t getMembership(std::double_t y) final;
};
