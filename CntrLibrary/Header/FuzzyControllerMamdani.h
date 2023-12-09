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
