#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <vector>

#include "FuzzyController.h"
#include "FuzzyControllerFactory.h"
#include "TriangularFuzzySet.h"
#include "TrapezoidalIFuzzySetInfL.h"
#include "TrapezoidalIFuzzySetInfR.h"
#include "FuzzyInput.h"
#include "FuzzyOutput.h"
#include "FisFileImport.h"
#include "FisFileExport.h"

#include "StringUtil.h"

using namespace CntrlLibrary;

TEST(TestCaseFuzzySet, FuzzySet1)
{
	std::unique_ptr<FuzzyInput> temperature = std::make_unique<FuzzyInput>(-80.00, 80.00, "Temperature");

	EXPECT_EQ(-80.00, temperature.get()->getMinimum());
	EXPECT_EQ(80.00, temperature.get()->getMaximum());
	EXPECT_EQ("Temperature", temperature.get()->getName());

	std::unique_ptr<FuzzySet> veryLow = std::make_unique<TrapezoidalIFuzzySetInfL>(-15.0, -3.0, "VeryLow");
	std::unique_ptr<FuzzySet> low = std::make_unique<TriangularFuzzySet>(-5.0, 0.0, 18.0, "Low");

	EXPECT_EQ(0.58333333333333337, veryLow->getMembership(-10.0));
	EXPECT_EQ(1.0, veryLow->getMembership(-60.0));
	EXPECT_EQ(0.0, veryLow->getMembership(0.0));

	EXPECT_EQ(0.0, low->getMembership(-7.0));
	EXPECT_EQ(1.0, low->getMembership(0.0));

	EXPECT_EQ(1.0, low->getMembership(0.0));
	EXPECT_EQ(0.97777777777777786, low->getMembership(0.4));
		
}
using std::experimental::filesystem::path;

TEST(TestCaseName, TestName)
{
	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();
	
	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	strpath = strpath + "test/test.fis";

	FisFileImport fis(strpath);
	FuzzyController* controllerFromFis = nullptr;
	if (false == fis.readFisFile(ossErrors))
	{
		std::cout << ossErrors.str();
	}
	else
	{
		controllerFromFis = fis.toFuzzyController();
	}

	EXPECT_EQ(1.0, 1.0);
}



TEST(CompareWithReference, TestTank)
{

	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/tank.fis";
	std::string fileName2 = strpath + "test/tank.txt";

	FisFileImport fis(fileName1);
	FuzzyController* controllerFromFis = nullptr;
	if (false == fis.readFisFile(ossErrors))
	{
		std::cout << ossErrors.str();
	}
	else
	{
		controllerFromFis = fis.toFuzzyController();
	}

	FuzzyInput* level = controllerFromFis->getInput("level");
	FuzzyInput* rate = controllerFromFis->getInput("rate");
	FuzzyOutput* valve = controllerFromFis->getOutput();

	controllerFromFis->compile();

	std::ifstream file(fileName2);

	if (!file)
	{
		ossErrors << "Error opening file " << std::endl;
	}
	std::string line;

	std::vector<std::double_t> in1;
	std::vector<std::double_t> in2;
	std::vector<std::double_t> ref;
	std::vector<std::double_t> is;

	while (std::getline(file, line))
	{
		std::vector<std::string> tokens = StringUtil::split(line, "\t");
		if (tokens.size() == 3U)
		{
			std::double_t i1 = std::stod(tokens[0]);
			std::double_t i2 = std::stod(tokens[1]);
			std::double_t r1 = std::stod(tokens[2]);
			
			in1.push_back(i1);
			in2.push_back(i2);
			level->setValue(i1);
			rate->setValue(i2);

			ref.push_back(r1);

			controllerFromFis->process();

			std::double_t o1 = valve->getValue();
			is.push_back(o1);
			
			std::double_t diff = abs( o1 - r1 );
			if (diff > 0.0002)
			{
				EXPECT_FLOAT_EQ(diff, 0);
			}
		}
	}
}

TEST(TestImportExport, TestTankImportExport)
{
	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/tank.fis";
	std::string fileName2 = strpath + "test/tank_exported.fis";

	FisFileImport fis(fileName1);
	FuzzyController* controllerFromFis = nullptr;
	if (false == fis.readFisFile(ossErrors))
	{
		std::cout << ossErrors.str();
	}
	else
	{
		controllerFromFis = fis.toFuzzyController();
	}

	std::ofstream file(fileName2);

	if (!file)
	{
		ossErrors << "Error creating file " << std::endl;
	}

	std::ifstream file1(fileName1);

	FisFileExport fexp(controllerFromFis, file);

	EXPECT_TRUE(fexp.exportToFIS());
	//to do compare two files

	file.close();

	std::ifstream file2(fileName2);

	file2.seekg(0, std::ifstream::beg);
	file1.seekg(0, std::ifstream::beg);
	bool equ =  std::equal(std::istreambuf_iterator<char>(file2.rdbuf()),
		std::istreambuf_iterator<char>(),
		std::istreambuf_iterator<char>(file1.rdbuf()));

	EXPECT_TRUE(equ);
}


TEST(TestImportExport, TestSugenoImportExport1)
{
	std::ostringstream ossErrors;
	
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/sugeno1.fis";
	std::string fileName2 = strpath + "test/sugeno1_exported.fis";

	FisFileImport fis(fileName1);
	FuzzyController* controllerFromFis = nullptr;
	if (false == fis.readFisFile(ossErrors))
	{
		std::cout << ossErrors.str();
	}
	else
	{
		controllerFromFis = fis.toFuzzyController();
	}

	std::ofstream file(fileName2);

	if (!file)
	{
		ossErrors << "Error creating file " << std::endl;
	}

	std::ifstream file1(fileName1);

	FisFileExport fexp(controllerFromFis, file);

	EXPECT_TRUE(fexp.exportToFIS());
	//to do compare two files

	file.close();

	std::ifstream file2(fileName2);

	file2.seekg(0, std::ifstream::beg);
	file1.seekg(0, std::ifstream::beg);
	bool equ = std::equal(std::istreambuf_iterator<char>(file2.rdbuf()),
		std::istreambuf_iterator<char>(),
		std::istreambuf_iterator<char>(file1.rdbuf()));

	EXPECT_TRUE(equ);
}




TEST(TestImportExport, TestInput1)
{
	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/test_input1.fis";
	std::string fileName2 = strpath + "test/test_input1_exported.fis";

	FisFileImport fis(fileName1);
	FuzzyController* controllerFromFis = nullptr;
	if (false == fis.readFisFile(ossErrors))
	{
		std::cout << ossErrors.str();
	}
	else
	{
		controllerFromFis = fis.toFuzzyController();
	}

	std::ofstream file(fileName2);

	if (!file)
	{
		ossErrors << "Error creating file " << std::endl;
	}

	std::ifstream file1(fileName1);

	FisFileExport fexp(controllerFromFis, file);

	EXPECT_TRUE(fexp.exportToFIS());
	//to do compare two files

	file.close();

	std::ifstream file2(fileName2);

	file2.seekg(0, std::ifstream::beg);
	file1.seekg(0, std::ifstream::beg);
	bool equ = std::equal(std::istreambuf_iterator<char>(file2.rdbuf()),
		std::istreambuf_iterator<char>(),
		std::istreambuf_iterator<char>(file1.rdbuf()));

	EXPECT_TRUE(equ);
}

TEST(TestCreateAndExport, TestCreate)
{
	/*define input variables */

	std::unique_ptr<FuzzyInput> temperature = std::make_unique<FuzzyInput>(-80.00, 80.00, "Temperature");

	std::unique_ptr<FuzzySet> veryLow = std::make_unique<TrapezoidalIFuzzySetInfL>(-15.0, -3.0, "VeryLow");
	std::unique_ptr<FuzzySet> low = std::make_unique<TriangularFuzzySet>(-5.0, 0.0, 18.0, "Low");
	std::unique_ptr<FuzzySet> normal = std::make_unique<TriangularFuzzySet>(18, 24, 30, "Normal");
	std::unique_ptr<FuzzySet> high = std::make_unique<TriangularFuzzySet>(28.0, 40.0, 52.0, "High");
	std::unique_ptr<FuzzySet> veryHigh = std::make_unique<TrapezoidalIFuzzySetInfR>(50.0, 60.0, "VeryHigh");

	temperature->addFuzzySet(std::move(veryLow));
	temperature->addFuzzySet(std::move(low));
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

	std::vector<std::string> rv = { "Temperature", "Voltage" };

	// if (Temperature is more or less VeryLow) or (Voltage is Low) then PWM is Low
	std::vector<std::vector<std::string>> vecHedgesR1;
	std::vector<std::string> vecHedgesR1T;
	std::vector<std::string> vecHedgesR1V;
	vecHedgesR1T.push_back("is");
	vecHedgesR1T.push_back("more or less");

	vecHedgesR1V.push_back("is");

	vecHedgesR1.push_back(vecHedgesR1T);
	vecHedgesR1.push_back(vecHedgesR1V);

	std::vector<std::string> rl1 = { "VeryLow",		"Low" };
	std::vector<std::string> rl2 = { "Low",			"Low" };
	std::vector<std::string> rl3 = { "Normal",			"Low" };
	std::vector<std::string> rl4 = { "VeryLow",			"Normal" };
	std::vector<std::string> rl5 = { "Low",				"Normal" };
	std::vector<std::string> rl6 = { "Normal",			"Normal" };
	std::vector<std::string> rl7 = { "High",			"Normal" };

	controller->addRule(BooleanOperation::AndMin, "R1", "", rv, vecHedgesR1, rl1, "PWM", "Low", 1.00);

	// if (Temperature is VeryLow) and (Voltage is Normal) then PWM is High
	std::vector<std::vector<std::string>> vecHedgesR2;
	std::vector<std::string> vecHedgesR2T;
	std::vector<std::string> vecHedgesR2V;
	vecHedgesR2T.push_back("is");
	vecHedgesR2V.push_back("is");
	vecHedgesR2.push_back(vecHedgesR2T);
	vecHedgesR2.push_back(vecHedgesR2V);

	controller->addRule(BooleanOperation::AndMin, "R2", "", rv, vecHedgesR2, rl2, "PWM", "Low", 1.00);

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
	std::double_t v1 = 20.00;

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