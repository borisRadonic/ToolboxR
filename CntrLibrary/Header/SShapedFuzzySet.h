#pragma once

#include "FuzzySet.h"

class SShapedFuzzySet final : public FuzzySet
{
public:

	SShapedFuzzySet() = delete;

	SShapedFuzzySet(std::double_t a, std::double_t b, const std::string& name = "");

	virtual ~SShapedFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::string getMSFTypeNameFIS() final;

	virtual std::string getMSFParamExportFISString() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_a;
	std::double_t m_b;
};