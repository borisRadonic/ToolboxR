#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <vector>

#include "StateSpace.h"

#include "DCMotor.h"
#include "FrictionModelCV.h"
#include "PIDController.h"

#include "StringUtil.h"
#include "Eigen\core"

using namespace CntrlLibrary;
using namespace Models;

using std::experimental::filesystem::path;

using namespace DiscreteTime;

TEST(TestStateSpaceClass, TestSS)
{
	StateSpace<2, 2, 2> sp;
	StateSpace<2, 2, 2>::AMatrix A;
	StateSpace<2, 2, 2>::AMatrix B;
	StateSpace<2, 2, 2>::AMatrix C;
	StateSpace<2, 2, 2>::AMatrix D;
	A(0) = -0.11;
	A(1) =  0.12;
	A(2) = -1.1;
	A(3) = -1.2;

	B(0) = 1.0;
	B(1) = 0.0;
	B(2) = 0.0;
	B(3) = 0.0;

	C.setIdentity();
	D.setZero();
	
	Eigen::Vector<double, 2> u;
	u(0) = 1.0;
	u(1) = 0.0;

	sp.setParameters(A, B, C, D, "");

	sp.setInput(u);

	sp.process();

	Eigen::Vector<double, 2> y;

	y = sp.getY();

	//sp.setParameters( )
}
/*
TEST(TestCaseDCMotor, TestDCMotorWithFriction)
{
	DCMotor motor;
	std::double_t J = 0.008586328125;

	std::double_t Ki1 = 1904.96918720126;
	std::double_t Ki2 = 30.2;

	motor.setParameters(0.0001, 0.000135, 0.178, J, 1.1, 1.55e-3, 0.7614);
	
	PIDController piTq;
	piTq.setParameters(2.00174495936295, Ki1, 0.00, Ki1, 0.0001, 120.0);


	PIDController piVel;
	piVel.setParameters(2.0, Ki2, 0.00, Ki2, 0.0001, 40.0);

	FrictionModelCSV friction;
	friction.setParameters(0.0001, 0.010, 3.5, J);

	//simulate first 0.1 s
	std::double_t I = 0.00;
	std::double_t w = 0.00;
	std::double_t a = 0.00;
	std::double_t T = 0.00;
	std::double_t Tf = 0.00;
	std::double_t Tref = 0.0;
	std::double_t error = 0.00;
	std::double_t errorVel = 0;
	std::double_t refVel = 100.00;
	std::double_t u = 0.00;

	std::vector< std::double_t> vecVel;
	std::vector< std::double_t> vecU;
	std::vector< std::double_t> vecTr;
	std::vector< std::double_t> vecTime;
	std::double_t time = 0.00;

	for (std::uint32_t k = 0; k < 10000; k++)
	{
		//velocity controller
		errorVel = refVel - w;
		Tref = piVel.process(errorVel);

		vecTr.push_back(Tref);
		
		//torque controller
		error = Tref - T;
		u = piTq.process(error);
				
		motor.setInputs( u, Tf);
		motor.process();
		I = motor.getCurrent();
		w = motor.getVelocity();
		a = motor.getAccell();
		T = motor.getTorque();
		
		vecVel.push_back(w);
		vecU.push_back(u);

		vecTime.push_back(time);
		time += 0.0001;
		
		//friction.setInputs(w, T, a);
		//friction.process();
		//Tf = friction.getFrictionTorque();
	}
	

	//EXPECT_FLOAT_EQ(i, 26.309461273007749);
	//EXPECT_FLOAT_EQ(w, 514.91089859268288);

	

	//EXPECT_FLOAT_EQ(i, 0.11944857346561119);
	//EXPECT_FLOAT_EQ(w, 673.41914414630844);
}
*/

TEST(TestCaseDCMotor, TestDCMotor1)
{
	DCMotor motor;
	motor.setParameters(0.0001, 0.000135, 0.178, 0.008586328125, 1.1, 1.55e-3, 0.7614);

	//simulate first 0.1 s
	motor.setInputs(120.00, 0.00);
	for (std::uint32_t i = 0; i < 1000; i++)
	{
		motor.process();
	}
	std::double_t i = motor.getCurrent();
	std::double_t w = motor.getVelocity();

	EXPECT_FLOAT_EQ(i, 26.309461273007749);
	EXPECT_FLOAT_EQ(w, 514.91089859268288);

	//next 0.9 s
	for (std::uint32_t i = 0; i < 9000; i++)
	{
		motor.process();
	}

	i = motor.getCurrent();
	w = motor.getVelocity();

	EXPECT_FLOAT_EQ(i, 0.11944857346561119);
	EXPECT_FLOAT_EQ(w, 673.41914414630844);
}



TEST(TestCaseDCMotor, TestPIDwithDCMotor)
{
	DCMotor motor;
	motor.setParameters(0.0001, 0.000135, 0.178, 0.008586328125, 1.1, 1.55e-3, 0.7614);

	//simulate first 0.1 s
	motor.setInputs(120.00, 0.00);
	for (std::uint32_t i = 0; i < 1000; i++)
	{
		motor.process();
	}
	std::double_t i = motor.getCurrent();
	std::double_t w = motor.getVelocity();

	EXPECT_FLOAT_EQ(i, 26.309461273007749);
	EXPECT_FLOAT_EQ(w, 514.91089859268288);

	//next 0.9 s
	for (std::uint32_t i = 0; i < 9000; i++)
	{
		motor.process();
	}

	i = motor.getCurrent();
	w = motor.getVelocity();

	EXPECT_FLOAT_EQ(i, 0.11944857346561119);
	EXPECT_FLOAT_EQ(w, 673.41914414630844);
}
