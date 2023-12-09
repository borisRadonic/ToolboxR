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

#include "FuzzyController.h"
#include "FuzzyTypes.h"
#include <memory>
#include <exception>

namespace CntrlLibrary
{
	FuzzyController::FuzzyController() :_controllerType(FuzzyControllerType::Mamdani)
	{
	}

	FuzzyController::~FuzzyController()
	{
	}

	FuzzyController::FuzzyController(const FuzzyControllerType& type, const std::string& name)
		:_controllerType(type), _controllerName(name)
	{
		if (type == FuzzyControllerType::Mamdani)
		{
			_ptrRules = std::make_unique<MamdaniRules>();
		}
		else
		{
			_ptrRules = std::make_unique<SugenoRules>();
		}
	}

	void FuzzyController::addInput(std::unique_ptr<FuzzyInput> input)
	{
		if (nullptr != input)
		{
			_inputs.push_back( std::move(input) );
		}
	}

	FuzzyInput* FuzzyController::getInput(const std::string& inputName)
	{
		for (auto& in : _inputs)
		{
			if( in.get()->getName() == inputName )
			{
				return in.get();
			}
		}
		return nullptr;
	}

	bool FuzzyController::setInputValue(const std::string& inputName, const std::double_t value)
	{
		for (auto& in : _inputs)
		{
			if (in.get()->getName() == inputName)
			{
				in->setValue(value);
			}
		}
		return false;
	}

	std::vector<std::string> FuzzyController::getInputs()
	{
		std::vector<std::string> vec;

		for (auto& in : _inputs)
		{
			vec.push_back(in.get()->getName());
		}
		return vec;
	}

	void FuzzyController::addOutput(std::unique_ptr<FuzzyOutput> output)
	{
		if (nullptr != output)
		{
			_output = std::move(output);
		}
	}

	std::vector<std::string> FuzzyController::getOutputs()
	{
		std::vector<std::string> vec;
		vec.push_back(_output->getName());
		return vec;
	}

	FuzzyOutput* FuzzyController::getOutput()
	{
		return _output.get();
	}

	const std::double_t FuzzyController::getOutputValue(const std::string& outputName)
	{
		return _output->getValue();
	}

	void FuzzyController::process()
	{
		/*process rules*/
		getOutput()->setValue(_ptrRules->process());
	}

	void FuzzyController::compile()
	{
		/*compile the rules*/
		for (size_t i = 0; i < _ptrRules->getNumberOfRules(); i++)
		{
			Rule* rule = _ptrRules->getRule(i);
			_ptrRules->compile();
		}
	}
}