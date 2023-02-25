#include "FuzzySet.h"

//The equation for the fuzzy Small function is:   1/ (1+(x/midpoint)^-spread)
//in midpoint u= 0.5
//Increasing the spread causes the fuzzy membership curve to become steeper.

class LargeFuzzySet final : public FuzzySet
{
public:
	LargeFuzzySet() = delete;

	LargeFuzzySet(std::double_t midpoint, std::double_t spread, const std::string& name = "");

	virtual ~LargeFuzzySet() {}

	virtual FuzzyMembershipFunctionType getMSFType() final;

	virtual std::string getMSFTypeNameFIS() final;

	virtual std::string getMSFParamExportFISString() final;

	virtual std::double_t getMembership(std::double_t y) final;

private:
	std::double_t m_midpoint;
	std::double_t m_spread;
};
