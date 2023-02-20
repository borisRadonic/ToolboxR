#pragma once
#include "FuzzySet.h"

//1 in(b, inf], it rises linearly from 0 to 1 in(a, b)

class TrapezoidalIFuzzySetInfR final : public FuzzySet
{
public:

	TrapezoidalIFuzzySetInfR() = delete;

	TrapezoidalIFuzzySetInfR(std::double_t a, std::double_t b, const std::string& name = "");

	virtual ~TrapezoidalIFuzzySetInfR() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_a;
	std::double_t m_b;
};