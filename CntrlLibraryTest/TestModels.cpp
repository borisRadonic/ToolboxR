#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <vector>

#include "StateSpace.h"

#include "DCMotor.h"
#include "HBPWMDCMotor.h"
#include "FrictionModelCV.h"
#include "PIDController.h"
#include "WaveFormTracer.h"

#include "StringUtil.h"
#include "Eigen\core"

using namespace CntrlLibrary;
using namespace Models;

using std::experimental::filesystem::path;

using namespace DiscreteTime;

TEST(TestStateSpaceClass, TestSS)
{

	/*
	A must be an N-by-N matrix, where n is the number of states. -> State transition matrix
	B must be an N-by-M matrix, where m is the number of inputs.
	C must be an R-by-N matrix, where r is the number of outputs.
	D must be an R-by-M matrix
	*/

	//StateSpace<N, M, R> sp;

	StateSpace<2, 2, 1> sp;
		
	using SS = StateSpace<2, 2, 1>;
	
	SS::AMatrix A;
	SS::BMatrix B;
	SS::CMatrix C;
	SS::DMatrix D;
	A(0) = 0.9315;
	A(1) = -0.01109;
	A(2) = 0.000856;
	A(3) = 1.0;

	B(0) = 0.06012;
	B(1) = 0.00;



	C.setIdentity();
	D.setZero();

	
	Eigen::Vector<double, 2> u;
	u(0) = 1.0;
	u(1) = 0.0;

	sp.setParameters(A, B, C, D, "");

	sp.setInput(u);

	Eigen::Vector<double, 1> y;

	//1 sec
	for (std::uint32_t k = 0; k < 60000; k++)
	{

		sp.process();
		y = sp.getY();
	}

	
	y = sp.getY();
	
	

}
/*
TEST(TestCaseDCMotor, TestDCMotorWithFriction)
{
	DCMotor motor;

	std::double_t Ki1 = 33.2;
	std::double_t Ki2 = 30.2;

	motor.setParameters(0.0001, 0.000135, 0.178, 0.08586328125, 1.1, 1.55e-3, 0.7614);

	
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
	motor.setParameters(0.0001, 0.000135, 0.178, 0.08586328125, 1.1, 1.55e-3, 0.7614);
	
	std::vector< std::double_t> vecVel;

	//simulate first  s
	motor.setInputs(120.00, 0.00);
	for (std::uint32_t i = 0; i < 20000; i++)
	{
		motor.process();
		vecVel.push_back(motor.getVelocity());
	}
	std::double_t i = motor.getCurrent();
	std::double_t w = motor.getVelocity();

	EXPECT_FLOAT_EQ(i, 6.2671933654315852);
	EXPECT_FLOAT_EQ(w, 635.50450403444040);

	//next 0.9 s
	for (std::uint32_t i = 0; i < 9000; i++)
	{
		motor.process();
	}

	i = motor.getCurrent();
	w = motor.getVelocity();

	//EXPECT_FLOAT_EQ(i, 0.11944857346561119);
	//EXPECT_FLOAT_EQ(w, 673.41914414630844);
}



TEST(TestCaseDCMotor, TestDCMotor2)
{
	DCMotor motor;
	motor.setParameters(0.0001, 0.000135, 0.178, 0.008586328125, 1.1, 1.55e-3, 0.7614);


	PIDController piVel;

	std::double_t kp = 2.00174495936295;
	std::double_t ki = 100.5;
	std::double_t kd = 0.00; 
	std::double_t kb = 0.00;
	std::double_t ts = 0.0001;
	std::double_t upSaturation = 120.00;
	piVel.setParameters( kp, ki, kd, kb, ts, upSaturation);

	double error = 0.0;
	

	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestDCMotor2.dat";

	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());
		

	tracer.addBlockSignal(piVel.getInputSignal(0));
	tracer.addBlockSignal(piVel.getOutputSignal(0));

	auto refVelShPtr = tracer.addSignal<std::double_t>("refVel", BaseSignal::SignalType::Double);
	auto motorVelShPtr = tracer.addSignal<std::double_t>("Vel", BaseSignal::SignalType::Double);

	tracer.writeHeader();

	refVelShPtr->set(0.5);
		
	for (std::uint32_t i = 0; i < 1000; i++)
	{
		motorVelShPtr->set(motor.getVelocity());
		error = refVelShPtr->get() - motorVelShPtr->get();
		motor.setInputs(piVel.process(error), 0.00);
		motor.process();
		tracer.trace();
	}
	std::double_t i = motor.getCurrent();
	std::double_t w = motor.getVelocity();

	//EXPECT_FLOAT_EQ(i, 26.309461273007749);
	//EXPECT_FLOAT_EQ(w, 514.91089859268288);

}


TEST(TestCasePWMotor, TestDCMotorPWM)
{
	HBPWMDCMotor motor;
	//Based on IGBT characteristics of Infineon IHW20N65R5
	std::double_t ts = 0.000001; //in this case 1us
	std::double_t tsc = 0.0001; //100 us

	std::double_t Vfdi = 0.8;
	std::double_t SlopeDi = 17.77;
	std::double_t Vfce = 0.75;
	std::double_t SlopeIc = 20.0;

	motor.setParameters(ts, 0.000135, 0.178, 0.008586328125, 1.1, 1.55e-3, 0.7614,Vfdi, SlopeDi, Vfce,SlopeIc );


	PIDController piVel;

	std::double_t kp = 5.1;
	std::double_t ki = 0.0;
	std::double_t kd = 0.00;
	std::double_t kb = 0.00;

	
	
	std::double_t upSaturation = 12.00;
	piVel.setParameters(kp, ki, kd, kb, tsc, upSaturation);

	double error = 0.0;


	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestDCMotorPWM.dat";

	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());


	tracer.addBlockSignal(piVel.getInputSignal(0));
	tracer.addBlockSignal(piVel.getOutputSignal(0));

	auto refVelShPtr = tracer.addSignal<std::double_t>("refVel", BaseSignal::SignalType::Double);
	auto motorVelShPtr = tracer.addSignal<std::double_t>("Vel", BaseSignal::SignalType::Double);

	tracer.writeHeader();

	refVelShPtr->set(50.5);

	std::double_t dcBusVoltage = upSaturation; //const at the moment

	for (std::uint32_t i = 0U; i < 10000U; i++)
	{
		motorVelShPtr->set(motor.getVelocity());
		error = refVelShPtr->get() - motorVelShPtr->get();
		//calculate duty cycle
		std::double_t val = piVel.process(error);
		std::double_t duty_cycle = val / upSaturation;
		//PWM in the midle of cycle
		std::int32_t count = (std::uint32_t) (duty_cycle * 100);
		std::uint32_t start = 50 - abs(count / 2);
		std::uint32_t end = 50 + abs(count / 2);
	
		for (std::uint32_t i = 0; i < 20; i++)
		{
			if (i < start || i >= end)
			{
				motor.setInputs(dcBusVoltage, 0.00, false,false);
			}
			else
			{
				if (val >= 0.0)
				{
					motor.setInputs(dcBusVoltage, 0.00, true, false);
				}
				else
				{
					motor.setInputs(dcBusVoltage, 0.00, false, true);
				}
			}
			tracer.trace();
			motor.process();
		}		
	}
	std::double_t i = motor.getCurrent();
	std::double_t w = motor.getVelocity();

	//EXPECT_FLOAT_EQ(i, 26.309461273007749);
	//EXPECT_FLOAT_EQ(w, 514.91089859268288);

}
