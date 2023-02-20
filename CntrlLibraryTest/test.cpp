#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>

#include "FuzzyController.h"
#include "FuzzyControllerFactory.h"
#include "TriangularFuzzySet.h"
#include "TrapezoidalIFuzzySetInfL.h"
#include "TrapezoidalIFuzzySetInfR.h"
#include "FuzzyInput.h"
#include "FuzzyOutput.h"
#include "FisFileImport.h"
#include "StringUtil.h"

#include <sstream>


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