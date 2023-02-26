// TestApp1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <memory>
#include <vector>
#include <sstream>

#include "FuzzyController.h"
#include "FuzzyControllerFactory.h"
#include "TriangularFuzzySet.h"
#include "TrapezoidalIFuzzySetInfL.h"
#include "TrapezoidalIFuzzySetInfR.h"
#include "FuzzyInput.h"
#include "FuzzyOutput.h"
#include "FisFileImport.h"
#include "StringUtil.h"


using namespace CntrlLibrary;

int main()
{

	/*define input variables */

	std::unique_ptr<FuzzyInput> temperature = std::make_unique<FuzzyInput>(-80.00, 80.00, "Temperature" );

	std::unique_ptr<FuzzySet> veryLow = std::make_unique<TrapezoidalIFuzzySetInfL>(-15.0, -3.0, "VeryLow");
	std::unique_ptr<FuzzySet> low = std::make_unique<TriangularFuzzySet>(-5.0, 0.0, 18.0, "Low");
	std::unique_ptr<FuzzySet> normal	= std::make_unique<TriangularFuzzySet>(18, 24, 30, "Normal");
	std::unique_ptr<FuzzySet> high		= std::make_unique<TriangularFuzzySet>(28.0, 40.0, 52.0, "High");
	std::unique_ptr<FuzzySet> veryHigh = std::make_unique<TrapezoidalIFuzzySetInfR>(50.0, 60.0, "VeryHigh");
		   
	temperature->addFuzzySet( std::move(veryLow) );
	temperature->addFuzzySet( std::move(low) );
	temperature->addFuzzySet(std::move(normal));
	temperature->addFuzzySet(std::move(high));
	temperature->addFuzzySet(std::move(veryHigh));

	std::unique_ptr<FuzzyInput> dcBusVoltage = std::make_unique<FuzzyInput>(-10.0, 900.00, "Voltage");

	std::unique_ptr<FuzzySet> veryLowVoltage = std::make_unique<TrapezoidalIFuzzySetInfL>(0.0, 40, "Low");
	std::unique_ptr<FuzzySet> normalVoltage = std::make_unique<TriangularFuzzySet>(30, 120, 200, "Normal");
	std::unique_ptr<FuzzySet> highVoltage = std::make_unique<TrapezoidalIFuzzySetInfR>(195, 240, "High");

	dcBusVoltage->addFuzzySet(std::move(veryLowVoltage));
	dcBusVoltage->addFuzzySet(std::move(normalVoltage));
	dcBusVoltage->addFuzzySet(std::move(highVoltage));

	std::unique_ptr<FuzzyOutput> pwmSignal = std::make_unique<FuzzyOutput>(0.0, 1.00, "PWM");

	std::unique_ptr<FuzzySet> veryLowOutput = std::make_unique<TrapezoidalIFuzzySetInfL>(0.0, 0.3, "Low");
	std::unique_ptr<FuzzySet> normalOutput = std::make_unique<TriangularFuzzySet>(0.25, 0.5, 0.75, "Normal");
	std::unique_ptr<FuzzySet> highOutput = std::make_unique<TrapezoidalIFuzzySetInfR>(0.7, 1.0, "High");

	pwmSignal->addFuzzySet(std::move(veryLowOutput));
	pwmSignal->addFuzzySet(std::move(normalOutput));
	pwmSignal->addFuzzySet(std::move(highOutput));

	FuzzyController* controller = FuzzyControllerFactory::createController(FuzzyControllerType::Mamdani, "Fuzzy Logic PID Controller");

	controller->addInput(std::move(temperature));
	controller->addInput(std::move(dcBusVoltage));
	controller->addOutput(std::move(pwmSignal));

	std::vector<std::string> rv =  { "Temperature", "Voltage" };
	
	// if (Temperature is more or less VeryLow) or (Voltage is Low) then PWM is Low
	std::vector<std::vector<std::string>> vecHedgesR1; 
	std::vector<std::string> vecHedgesR1T;
	std::vector<std::string> vecHedgesR1V;
	vecHedgesR1T.push_back("is");
	vecHedgesR1T.push_back("more or less");
	
	vecHedgesR1V.push_back("is");
	
	vecHedgesR1.push_back(vecHedgesR1T);
	vecHedgesR1.push_back(vecHedgesR1V);
		
	std::vector<std::string> rl1 =  { "VeryLow",		"Low" };
	std::vector<std::string> rl2 =	{ "Low",			"Low" };
	std::vector<std::string> rl3 = { "Normal",			"Low" };
	std::vector<std::string> rl4 = { "VeryLow",			"Normal" };
	std::vector<std::string> rl5 = { "Low",				"Normal" };
	std::vector<std::string> rl6 = { "Normal",			"Normal" };
	std::vector<std::string> rl7 = { "High",			"Normal" };

	controller->addRule(BooleanOperation::AndMin, "R1", "", rv, vecHedgesR1, rl1, "PWM", "Low",1.00);
	
	// if (Temperature is VeryLow) and (Voltage is Normal) then PWM is High
	std::vector<std::vector<std::string>> vecHedgesR2;
	std::vector<std::string> vecHedgesR2T;
	std::vector<std::string> vecHedgesR2V;
	vecHedgesR2T.push_back("is");
	vecHedgesR2V.push_back("is");
	vecHedgesR2.push_back(vecHedgesR2T);
	vecHedgesR2.push_back(vecHedgesR2V);
	
	controller->addRule(BooleanOperation::AndMin, "R2", "", rv, vecHedgesR2, rl2, "PWM", "Low",	1.00 );

	controller->addRule(BooleanOperation::AndMin, "R4", "", rv, vecHedgesR2, rl4, "PWM", "High", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R3", "", rv, vecHedgesR2, rl3, "PWM", "Low", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R5", "", rv, vecHedgesR2, rl5, "PWM", "High", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R7", "", rv, vecHedgesR2, rl7, "PWM", "Normal", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R6", "", rv, vecHedgesR2, rl6, "PWM", "High", 1.00);
	   	
		
	FuzzyInput* inTemp = controller->getInput("Temperature");
	FuzzyInput* inVolt = controller->getInput("Voltage");
	FuzzyOutput* outPWM = controller->getOutput();

	controller->compile();

	std::double_t t1 = -20;
	std::double_t v1 =  20.00;
	
	std::double_t val;

	struct RES
	{
		std::double_t temperatur;
		std::double_t voltage;
		std::double_t value;
	};
	std::vector<RES> results;
	RES res;
	for (int i = 0; i < 100; i++)
	{
		t1 = t1 + 1.0;
		v1 = v1 + 0.7;
		res.temperatur = t1;
		res.voltage = v1;
		inTemp->setValue(t1);
		inVolt->setValue(v1);
		controller->process();
		val = outPWM->getValue();
		res.value = val;
		results.push_back(res);
	}
}
