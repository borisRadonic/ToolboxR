#include "SingletonFuzzySet.h"
#include <format>

namespace CntrlLibrary
{
	FuzzyMembershipFunctionType SingletonFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::Singleton;
	}

	std::string SingletonFuzzySet::getMSFTypeNameFIS()
	{
		return std::string("singleton");
	}

	std::string SingletonFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_c) + "]");
	}

	std::double_t SingletonFuzzySet::getMembership(std::double_t y)
	{
		std::double_t m = MIN_DOUBLE_DIFF;
		std::double_t max = m_c + m;
		std::double_t min = m_c - m;

		if ((y >= min) && (y <= max))
		{
			return std::double_t(1.0);
		}
		return std::double_t(0.0);;
	}

	FuzzyMembershipFunctionType SingletonSugenoFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::SingletonSugeno;
	}

	std::string SingletonSugenoFuzzySet::getMSFTypeNameFIS()
	{
		return "constant";
	}

	std::string SingletonSugenoFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_c) + "]");
	}

	std::double_t SingletonSugenoFuzzySet::getMembership(std::double_t y)
	{
		//not used for saugeno system
		return std::double_t(1.00);
	}
	FuzzyMembershipFunctionType LinearSugenoFuzzySet::getMSFType()
	{
		return FuzzyMembershipFunctionType::LinearSugeno;
	}
	std::string LinearSugenoFuzzySet::getMSFTypeNameFIS()
	{
		return "linear";
	}
	std::string LinearSugenoFuzzySet::getMSFParamExportFISString()
	{
		return ("[" + std::format("{}", m_a) + " " + std::format("{}", m_b) + " " + std::format("{}", m_c) + "]");
	}
	std::double_t LinearSugenoFuzzySet::getMembership(std::double_t y)
	{
		//not used for saugeno system
		return std::double_t(1.00);
	}
}