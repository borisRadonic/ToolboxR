#pragma once
#include "FuzzySet.h"

class GaussianFuzzySet final : public FuzzySet
{
public:
	GaussianFuzzySet() = delete;

	GaussianFuzzySet(std::double_t midpoint, std::double_t spread, const std::string& name = "");

	virtual ~GaussianFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_midpoint;
	std::double_t m_spread;
};

