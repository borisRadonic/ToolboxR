#pragma once
#include "FuzzySet.h"

namespace CntrlLibrary
{
	/*
	SingletonSugenoFuzzySet Fuzzy set is used only for output and only with Sugeno  system
	*/
	class SingletonFuzzySet final : public FuzzySet
	{
	public:
		SingletonFuzzySet() = delete;

		SingletonFuzzySet(std::double_t c, const std::string& name = "") :m_c(c), FuzzySet(name) {}

		virtual ~SingletonFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;

		virtual std::double_t getMembership(std::double_t y) final;

	private:
		std::double_t m_c;
	};


	class SingletonSugenoFuzzySet final : public FuzzySet
	{
	public:
		SingletonSugenoFuzzySet() = delete;

		SingletonSugenoFuzzySet(std::double_t c, const std::string& name = "") :FuzzySet(name), m_c(c){}

		virtual ~SingletonSugenoFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;

		virtual std::double_t getMembership(std::double_t y) final;

		std::double_t getConstant() const
		{
			return m_c;
		}

	private:
		std::double_t m_c;
	
	};



	class LinearSugenoFuzzySet final : public FuzzySet
	{
	public:
		LinearSugenoFuzzySet() = delete;

		LinearSugenoFuzzySet(std::double_t a, std::double_t b, std::double_t c, const std::string& name = "") 
		:FuzzySet(name), m_a(a), m_b(b), m_c(c)
		{
		}

		virtual ~LinearSugenoFuzzySet() {}

		virtual FuzzyMembershipFunctionType getMSFType() final;

		virtual std::string getMSFTypeNameFIS() final;

		virtual std::string getMSFParamExportFISString() final;


		virtual std::double_t getMembership(std::double_t y) final;

		std::double_t get(std::double_t x, std::double_t y) const
		{
			return (m_a* x + m_b * y + m_c);
		}

	private:

		std::double_t m_a;
		std::double_t m_b;
		std::double_t m_c;

	};



};