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

#include "LinguisticVariable.h"

namespace CntrlLibrary
{
	LinguisticVariable::LinguisticVariable(const std::double_t minimum, const std::double_t maximum, const std::string& name)
		:_minimum(minimum), _maximum(maximum), _variableName(name), _value(0.0)
	{
	}

	LinguisticVariable::~LinguisticVariable()
	{
	}

	FuzzySet* LinguisticVariable::getFuzzySet(const std::string& name)
	{
		for (auto& lv : _lingvisticValues)
		{
			if (lv.get()->getName() == name)
			{
				return lv.get();
			}
		}
		return nullptr;
	}

	void LinguisticVariable::addFuzzySet(std::unique_ptr<FuzzySet> set)
	{
		if (nullptr != set)
		{
			std::string name = set->getName();

			for (auto& lv : _lingvisticValues)
			{
				if (lv.get()->getName() == name)
				{
					throw new std::exception("Already defined rule!");
				}
			}
			_lingvisticValues.push_back(std::move(set));
		}
	}

	void LinguisticVariable::setValue(const std::double_t value)
	{
		if (value < HUGE_VAL)
		{
			if ((value >= _minimum) && (value <= _maximum))
			{
				_value = value;
			}
			else
			{
				throw new std::exception("Input range violation");
			}
		}
		else
		{
			throw new std::exception("double NAN");
		}
	}

	std::double_t LinguisticVariable::getValue() const
	{
		return _value;
	}

	std::vector<std::string> LinguisticVariable::getLingvisticValues() const
	{
		std::vector<std::string> vec;
		for (auto& lv : _lingvisticValues)
		{
			vec.push_back(lv->getName());
		}
		return vec;
	}
}