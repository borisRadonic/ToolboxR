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
#include <vector>
#include <vector>
#include <map>
#include <memory>

#include "FuzzyHedges.h"
#include "FuzzyInput.h"
#include "FuzzyOutput.h"
#include "FuzzyTypes.h"

namespace CntrlLibrary
{
	class Rule
	{
	public:

		Rule();

		Rule(const std::string& name, const std::string& definition);

		Rule(const Rule&) = delete;

		Rule& operator=(Rule&) = delete;

		~Rule();

		void addHedge(const std::string& inputName, std::unique_ptr<FuzzyHedge> hedge);

		void addInput(FuzzyInput* input);

		void addOutput(FuzzyOutput* output);

		const std::string getName();

		std::double_t process();

		void addInputTerm(const std::string& term);

		inline void setBooleanType(BooleanOperation bolianType)
		{
			_boleanType = bolianType;
		}

		inline BooleanOperation getBooleanType()
		{
			return _boleanType;
		}

		inline void setOutputTerm(const std::string& term)
		{
			_outputTerm = term;
		}

		inline const std::string& getOutputTerm() const
		{
			return _outputTerm;
		}

		inline FuzzyOutput* getFuzzyOutput() const
		{
			return _ptrOutput;
		}

		inline void setControllerType(FuzzyControllerType type)
		{
			_controllerType = type;
		}

		inline void setWeight(std::double_t weight)
		{
			_weight = weight;
		}

		inline std::double_t getWeight() const
		{
			return _weight;
		}

		void compile();

		inline std::size_t getnumberOfInputs() const
		{
			return _inputTerms.size();
		}

		std::double_t getInputValue(const std::uint32_t i);


		inline const std::vector<std::string>& getInputTerms() const
		{
			return _inputTerms;
		}

	private:

		std::string _name;
		std::string _definition;

		using MapHedgesPtr = std::map< std::string, std::vector<std::unique_ptr<FuzzyHedge>>>;

		using InputPtr = std::vector<FuzzyInput*>;

		MapHedgesPtr _hedges; //should be access from end to beginn

		InputPtr _ptrInputs;

		std::vector<std::string> _inputTerms;

		FuzzyOutput* _ptrOutput = nullptr;

		std::string _outputTerm;

		BooleanOperation _boleanType = BooleanOperation::AndMin;

		FuzzyControllerType _controllerType = FuzzyControllerType::Mamdani;

		std::double_t _weight = 1.0;

	};
};
