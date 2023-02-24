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
#include "StringUtil.h"



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


