#pragma once

#include "FuzzySet.h"

class ZShapedFuzzySet final : public FuzzySet
{
public:

	ZShapedFuzzySet() = delete;

	ZShapedFuzzySet(std::double_t a, std::double_t b, const std::string& name = "");

	virtual ~ZShapedFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::string getMSFTypeNameFIS() final;

	virtual std::string getMSFParamExportFISString() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_a;
	std::double_t m_b;
};
