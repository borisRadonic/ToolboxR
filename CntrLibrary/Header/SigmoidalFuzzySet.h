#pragma once
#include "FuzzySet.h"

class SigmoidalFuzzySet final : public FuzzySet
{
public:
	SigmoidalFuzzySet() = delete;

	SigmoidalFuzzySet(std::double_t a, std::double_t c, const std::string& name = "");

	virtual ~SigmoidalFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::string getMSFTypeNameFIS() final;

	virtual std::string getMSFParamExportFISString() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:

	std::double_t m_a;
	std::double_t m_c;
};



