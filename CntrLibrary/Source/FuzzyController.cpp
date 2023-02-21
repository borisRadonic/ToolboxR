#include "FuzzyController.h"
#include "FuzzyTypes.h"
#include <memory>
#include <exception>

FuzzyController::FuzzyController():_controllerType(FuzzyControllerType::Mamdani)
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
	if( nullptr != input )
	{
		_inputs[input->getName()] = std::move(input);
	}	
}

FuzzyInput* FuzzyController::getInput(const std::string & inputName)
{
	if (_inputs.count(inputName) > 0U)
	{
		return _inputs[inputName].get();
	}
	return nullptr;
}

bool FuzzyController::setInputValue(const std::string & inputName, const std::double_t value)
{
	if (_inputs.count(inputName) > 0U)
	{
		_inputs[inputName]->setValue(value);
	}
	return false;
}

void FuzzyController::addOutput(std::unique_ptr<FuzzyOutput> output)
{
	if (nullptr != output)
	{
		_output = std::move(output);
	}
}

FuzzyOutput * FuzzyController::getOutput()
{
	return _output.get();
}

const std::double_t FuzzyController::getOutputValue(const std::string & outputName)
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

		int a = 0;
		i++;
	}
}
