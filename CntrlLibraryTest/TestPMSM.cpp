#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <vector>

#include "StateSpace.h"

#include "PMSMotor.h"
#include "PIDController.h"
#include "StringUtil.h"

using namespace CntrlLibrary;
using namespace Models;

using std::experimental::filesystem::path;

using namespace DiscreteTime;
TEST(TestSPMSClass, TestPMS)
{
	//PMSMotor motor;
	//motor.setParameters(0.0001, 0.000135, 0.178, 0.008586328125, 1.1, 1.55e-3, 0.7614);


	//motor.setInputs(120.00, 0.00);
	//for (std::uint32_t i = 0; i < 1000; i++)
	//{
		//motor.process();
	//}
	//std::double_t i = motor.getCurrent();
	//std::double_t w = motor.getVelocity();

	
}
