#pragma once

#include "FuzzySet.h"

class PiShapedFuzzySet final : public FuzzySet
{
public:

	PiShapedFuzzySet() = delete;

	PiShapedFuzzySet(std::double_t a, std::double_t b, std::double_t c, std::double_t d, const std::string& name = "");

	virtual ~PiShapedFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_a;
	std::double_t m_b;
	std::double_t m_c;
	std::double_t m_d;
};
