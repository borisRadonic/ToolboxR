

#pragma once
#include "FuzzySet.h"

// The generalized bell function depends on three parameters a, b and c as given by : f(x;a,b,midpoint) = 1/ ((1+ abs(x-midpoint))/ (2a)) ^2b)
// Midpoint determines the centre of the corresponding membership function
//a is the half width;
//a and b together control the slopes at the crossover points -> slope = b/2a

class BellShapedFuzzySet final : public FuzzySet
{
public:
	BellShapedFuzzySet() = delete;

	BellShapedFuzzySet(std::double_t midpoint, std::double_t width, std::double_t slope, const std::string& name = "");

	virtual ~BellShapedFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::double_t getMembership(std::double_t y) final;

	virtual std::double_t getFirstCore() final;

private:
	std::double_t m_midpoint;
	std::double_t m_a;
	std::double_t m_b;
};

