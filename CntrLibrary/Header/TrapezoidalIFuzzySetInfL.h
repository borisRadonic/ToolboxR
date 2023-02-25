#pragma once
#include "FuzzySet.h"

//1 in(-inf, a], it falls linearly from 1 to 0 in(a, b)

class TrapezoidalIFuzzySetInfL final : public FuzzySet
{
public:

	TrapezoidalIFuzzySetInfL() = delete;

	TrapezoidalIFuzzySetInfL(std::double_t a, std::double_t b, const std::string& name = "");

	virtual ~TrapezoidalIFuzzySetInfL() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::string getMSFTypeNameFIS() final;

	virtual std::string getMSFParamExportFISString() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_a;
	std::double_t m_b;
};

