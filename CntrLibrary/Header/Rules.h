/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

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

