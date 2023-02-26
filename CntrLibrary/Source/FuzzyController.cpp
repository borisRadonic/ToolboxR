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