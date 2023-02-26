#pragma once

#include <string>
#include <map>
#include <memory>

#include "Rule.h"

namespace CntrlLibrary
{

	class FuzzyController;

	class Rules
	{
	public:

		Rules();

		Rules(const Rules&) = delete;

		Rules& operator=(Rules&) = delete;

		virtual ~Rules();

		void setFuzzyController(FuzzyController* ptrController);

		void AddRule(std::unique_ptr<Rule> rule);

		std::size_t getNumberOfRules() const;

		std::vector<std::string> getNames();

		Rule* getRule(size_t i);

		Rule* getRule(std::string name);

		virtual std::double_t process();

		virtual void compile();

	protected:

		using MapRulePtr = std::map<std::string, std::unique_ptr<Rule>>;

		MapRulePtr _rules;

		std::vector <Rule*> _rulesOrderByOutTerm; //after compile

		std::vector<std::double_t> _vecDegOfActOrderByOutTerm; //for grupped rules

		FuzzyController* _ptrController = nullptr;

		bool _compiled = false;

		std::map<std::string, std::double_t> _vecOutTermDegOfAcivation; //for grupped rules
	};

	class MamdaniRules final : public Rules
	{
	public:

		MamdaniRules();

		MamdaniRules(const MamdaniRules&) = delete;

		MamdaniRules& operator=(MamdaniRules&) = delete;

		virtual ~MamdaniRules();

		virtual std::double_t process() override;

		virtual void compile() override;

	};


	class SugenoRules final : public Rules
	{
	public:

		SugenoRules();

		SugenoRules(const SugenoRules&) = delete;

		SugenoRules& operator=(SugenoRules&) = delete;

		virtual ~SugenoRules();

		virtual std::double_t process() override;

		virtual void compile() override;

	};
};

