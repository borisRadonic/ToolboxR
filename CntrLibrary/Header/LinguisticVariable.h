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
#include "vector"
#include "FuzzySet.h"

namespace CntrlLibrary
{
	class LinguisticVariable
	{
	public:

		using LingvisticValuesPtr = std::vector<std::unique_ptr<FuzzySet>>;

		LinguisticVariable() = delete;

		LinguisticVariable(const std::double_t minimum, const std::double_t maximum, const std::string& name);

		LinguisticVariable(const LinguisticVariable&) = delete;

		LinguisticVariable& operator=(LinguisticVariable&) = delete;

		virtual ~LinguisticVariable();

		const std::string getName()
		{
			return _variableName;
		}

		FuzzySet* getFuzzySet(const std::string& name);

		void addFuzzySet(std::unique_ptr<FuzzySet> set);

		void setValue(const std::double_t value);

		std::double_t getValue() const;

		std::double_t getMinimum() const
		{
			return _minimum;
		}

		std::double_t getMaximum() const
		{
			return _maximum;
		}

		std::vector<std::string> getLingvisticValues() const;


	protected:

		LingvisticValuesPtr _lingvisticValues;

		std::string _variableName;

		std::double_t _minimum;
		
		std::double_t _maximum;

		std::double_t _value = 0;
	};
}
