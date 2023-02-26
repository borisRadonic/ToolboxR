#pragma once
#include "FuzzyController.h"

namespace CntrlLibrary
{

	class FuzzyControllerMamdani final : public FuzzyController
	{
	public:

		FuzzyControllerMamdani();

		FuzzyControllerMamdani(const std::string& name);

		virtual ~FuzzyControllerMamdani();

		virtual void addRule(BooleanOperation booleanType,
			const std::string& name,
			const std::string& description, const std::vector<std::string>& inVariables,
			const std::vector<std::vector<std::string>>& hedges,
			const std::vector<std::string>& inLingValues,
			const std::string& outVariable,
			const std::string& outLingValue,			
			const std::double_t& weight) override; //Optional weight for Mamdani system


		virtual void process() override;

		virtual void compile() override;

	};
};
