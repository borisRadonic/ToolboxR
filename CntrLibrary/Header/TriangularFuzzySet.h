#pragma once
#include "FuzzySet.h"
class TriangularFuzzySet final : public FuzzySet
{
public:
	TriangularFuzzySet() = delete;

	TriangularFuzzySet(std::double_t a, std::double_t b, std::double_t c, const std::string& name = "");

	virtual ~TriangularFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_a;
	std::double_t m_b;
	std::double_t m_c;
};