#pragma once
#include "FuzzySet.h"

namespace CntrlLibrary
{
	class TrapezoidalFuzzySet final : public FuzzySet
	{
	public:
		TrapezoidalFuzzySet() = delete;

		TrapezoidalFuzzySet(std::double_t a, std::double_t b, std::double_t c, std::double_t d, const std::string& name = "");

		virtual ~TrapezoidalFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;

		virtual std::double_t getMembership(std::double_t y) final;

	private:
		std::double_t m_a;
		std::double_t m_b;
		std::double_t m_c;
		std::double_t m_d;
	};
}
