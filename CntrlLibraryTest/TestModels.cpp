#include "pch.h"


#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <vector>


#include "DCMotor.h"


#include "StringUtil.h"

using namespace CntrlLibrary;
using namespace Models;

using std::experimental::filesystem::path;

TEST(TestCaseDCMotor, TestDCMotor1)
{
	DCMotor motor;
	motor.setParameters(0.0001, 0.000135, 0.178, 0.008586328125, 1.1, 1.55e-3, 0.7614);
	
	//simulate first 0.1 s
	for (std::uint32_t i = 0; i < 1000; i++)
	{
		motor.process(120.00);
	}
	std::double_t i = motor.getCurrent();
	std::double_t w = motor.getVelocity();

	EXPECT_FLOAT_EQ(i, 26.309461273007749);
	EXPECT_FLOAT_EQ(w, 514.91089859268288);

	//next 0.9 s
	for (std::uint32_t i = 0; i < 9000; i++)
	{
		motor.process(120.00);
	}

	i = motor.getCurrent();
	w = motor.getVelocity();

	EXPECT_FLOAT_EQ(i, 0.11944857346561119);
	EXPECT_FLOAT_EQ(w, 673.41914414630844);
}

