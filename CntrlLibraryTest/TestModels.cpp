#include "pch.h"
#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <vector>

#include "StateSpace.h"

#include "DCMotor.h"
#include "HBPWMDCMotor.h"
#include "PMSMotor.h"
#include "PMSMPICurrentController.h"

#include "HexicPolynomial.h"

#include "JerkLimitedTrajectory.h"
#include "QuinticPolyTrajectory.h"

#include "FrictionModelCV.h"
#include "PIDController.h"
#include "WaveFormTracer.h"

#include "StringUtil.h"
#include "Eigen\core"

#include <Eigen/Dense>

using namespace CntrlLibrary;
using namespace Models;

using std::experimental::filesystem::path;

using namespace DiscreteTime;


TEST(TestStateSpaceClass, TestStateSpace)
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

TEST(TestCaseQuanticPolyTrajectory, TestCaseQuanticPolyTrajectoryBasic)
{
	using namespace TrajectoryGeneration;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/QuanticPolyTrajectory.dat";

	double ts = 0.001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto jerk = tracer.addSignal<std::double_t>("jerk", BaseSignal::SignalType::Double);
	auto acceleration = tracer.addSignal<std::double_t>("accel", BaseSignal::SignalType::Double);
	auto velocity = tracer.addSignal<std::double_t>("vel", BaseSignal::SignalType::Double);
	auto position = tracer.addSignal<std::double_t>("position", BaseSignal::SignalType::Double);

	double i_pos = -20.00;
	double i_vel = 10.00;
	double i_accel = 10.00;
	double f_pos = 300.00;
	double f_vel = 200.00;
	double f_accel = 15.00;
	double f_time = 1.50;
	double m_accel = 300.00;
	double m_vel = 300.00;

	/*
	double m_accel = 1000.00;
	double m_vel = 300.00;
	double i_pos = 0.00;
	double i_vel = 0.00;
	double i_accel = 0.00;
	double f_pos = 2.0;
	double f_vel = 0.00;
	double f_accel = 0.00;
	double f_time = 0.10;
	*/
	QuinticPolyTrajectory traj( i_pos,
								i_vel,
								i_accel,
								f_pos,
								f_vel,
								f_accel,								
								m_accel,
								m_vel );
		
	f_time = 0.1;
	if (QuinticPolyTrajectory::OptTimeResult::Converged != traj.calculateMinTime(f_time, 0.5, 0.5))
	{
		EXPECT_TRUE(false);
	}
	
	//EXPECT_TRUE(f_time > 0.001);
	traj.create(f_time);

	std::pair<double, double> ex_velocity;
	std::pair<std::pair<double, double>, std::pair<double, double>> ex_accelerations;
	Math::BasicNumMethods::ResultType result = traj.findExtremaNewtonRaphson( ex_velocity, ex_accelerations);

	double pos = 0.00;
	double vel = 0.00;
	double accel = 0.00;;
	double jrk = 0.00;;
	for (double t = 0.00001; t <= f_time; t = t + 0.001)
	{
		traj.calculateValuesForTime(t, pos, vel, accel, jrk);
		jerk->set(jrk/20.0);
		acceleration->set(accel);
		velocity->set(vel);
		position->set(pos);		
		tracer.trace();
	}
}

TEST(TestCaseJerkLimitedTrajectory, TestBasicJerkLimitedTrajectorySmallDistance)
{
	using namespace TrajectoryGeneration;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestTrajectory1.dat";

	double ts = 0.001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());


	auto acceleration	= tracer.addSignal<std::double_t>("accel", BaseSignal::SignalType::Double);
	auto velocity		= tracer.addSignal<std::double_t>("vel", BaseSignal::SignalType::Double);
	auto position		= tracer.addSignal<std::double_t>("position", BaseSignal::SignalType::Double);
	 

	JerkLimitedTrajectory traj;  


	double i_pos = 0.00;
	double i_vel = 0.00;
	double i_accel = 0.00;
	double f_pos = 0.2;
	double f_vel = 0.00;
	double f_accel = 0.00;
	double f_time = 0.10;
	

	traj.setParameters(20000.0, 1000.0, 300.00, 0.00001, 0.01, 0.1); // Max jerk, Max. acceleration, Max. velocity, max. pos. error, max vel. error and max. accel. error
	traj.setInitialValues( i_pos, i_vel, i_accel);
	traj.setFinalValues(f_pos, f_vel, f_accel);  // Target position, Target velocity, Target acceleration

	traj.prepare(f_time);

	double pos(0.00), vel(0.00), accel(0.00), jerk(0.00);
	for (double t = 0.00001; t <= f_time + 0.00001; t = t + 0.001)
	{
		traj.process(t, pos, vel, accel, jerk);
		acceleration->set(accel);
		velocity->set(vel);
		position->set(pos);
		tracer.trace();
	}

}


TEST(TestCaseJerkLimitedTrajectory, TestBasicJerkLimitedTrajectoryTrapezoidalProfile)
{
	using namespace TrajectoryGeneration;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestTrajectory2.dat";

	double ts = 0.001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());
	
	
	auto acceleration = tracer.addSignal<std::double_t>("accel", BaseSignal::SignalType::Double);
	auto velocity = tracer.addSignal<std::double_t>("vel", BaseSignal::SignalType::Double);
	auto position = tracer.addSignal<std::double_t>("position", BaseSignal::SignalType::Double);
	auto iposition = tracer.addSignal<std::double_t>("Iposition", BaseSignal::SignalType::Double);


	JerkLimitedTrajectory traj;

	double i_pos = 0.00;
	double i_vel = 0.00;
	double i_accel = 0.00;
	double f_pos = 500.;
	double f_vel = 0.00;
	double f_accel = 0.00;
	double f_time = 4.0;

	Integrator integral;
	integral.setParameters(IntegratorMethod::BackwardEuler, 0.001, 1.00);
	integral.setInitialConditions(i_vel);


	traj.setParameters(5000.0, 250.0, 300.00, 0.00001, 0.01, 0.1); // Max jerk, Max. acceleration, Max. velocity, max. pos. error, max vel. error and max. accel. error
	traj.setInitialValues(i_pos, i_vel, i_accel);
	traj.setFinalValues(f_pos, f_vel, f_accel);  // Target position, Target velocity, Target acceleration

	traj.prepare(f_time);

	double pos(0.00), vel(0.00), accel(0.00), jerk(0.00);
	for (double t = 0.00001; t <= f_time + 0.00001; t = t + 0.001)
	{
		traj.process(t, pos, vel, accel, jerk);
		acceleration->set(accel);
		velocity->set(vel);
		position->set(pos);
		iposition->set(integral.process(vel));
		tracer.trace();
	}

}


TEST(TestCaseJerkLimitedTrajectory, TestBasicJerkLimitedTrajectoryTriangularProfile)
{
	using namespace TrajectoryGeneration;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestTrajectory3.dat";

	double ts = 0.001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());


	auto acceleration = tracer.addSignal<std::double_t>("accel", BaseSignal::SignalType::Double);
	auto velocity = tracer.addSignal<std::double_t>("vel", BaseSignal::SignalType::Double);
	auto position = tracer.addSignal<std::double_t>("position", BaseSignal::SignalType::Double);


	JerkLimitedTrajectory traj;

	double i_pos = 0.00;
	double i_vel = 10.00;
	double i_accel = 0.00;
	double f_pos = 200.;
	double f_vel = 100.00;
	double f_accel = 0.00;
	double f_time = 4.0;


	traj.setParameters(5000.0, 250.0, 600.00, 0.00001, 0.01, 0.1); // Max jerk, Max. acceleration, Max. velocity, max. pos. error, max vel. error and max. accel. error
	traj.setInitialValues(i_pos, i_vel, i_accel);
	traj.setFinalValues(f_pos, f_vel, f_accel);  // Target position, Target velocity, Target acceleration

	traj.prepare(f_time);

	double pos(0.00), vel(0.00), accel(0.00), jerk(0.00);
	for (double t = 0.00001; t <= f_time + 0.00001; t = t + 0.001)
	{
		traj.process(t, pos, vel, accel, jerk);
		acceleration->set(accel);
		velocity->set(vel);
		position->set(pos);
		tracer.trace();
	}

}


TEST(TestCaseJerkLimitedTrajectory, TestBasicJerkLimitedTrajectorySafetyStop)
{
	using namespace TrajectoryGeneration;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestTrajectory4.dat";

	double ts = 0.001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());


	auto acceleration = tracer.addSignal<std::double_t>("accel", BaseSignal::SignalType::Double);
	auto velocity = tracer.addSignal<std::double_t>("vel", BaseSignal::SignalType::Double);
	auto position = tracer.addSignal<std::double_t>("position", BaseSignal::SignalType::Double);


	JerkLimitedTrajectory traj;

	double i_pos = 0.00;
	double i_vel = 100.00;
	double i_accel = 0.00;
	double f_pos = 0.2;
	double f_vel = 0.00;
	double f_accel = 0.00;
	double f_time = 4.0;


	traj.setParameters(5000.0, 500.0, 600.00, 0.00001, 0.01, 0.1); // Max jerk, Max. acceleration, Max. velocity, max. pos. error, max vel. error and max. accel. error
	traj.setInitialValues(i_pos, i_vel, i_accel);
	traj.setFinalValues(f_pos, f_vel, f_accel);  // Target position, Target velocity, Target acceleration

	traj.prepare(f_time);

	double pos(0.00), vel(0.00), accel(0.00), jerk(0.00);
	for (double t = 0.00001; t <= f_time + 0.00001; t = t + 0.001)
	{
		traj.process(t, pos, vel, accel, jerk);
		acceleration->set(accel);
		velocity->set(vel);
		position->set(pos);
		tracer.trace();
	}
}

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
	std::double_t kb = 1.00;
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

/*
TEST(TestCasePWMotor, TestDCMotorPWM)
{
	HBPWMDCMotor motor;
	//Based on dummy Transistor and diode charachteristic
	std::double_t ts = 0.000001; //in this case 1us
	std::double_t tsc = 0.0001; //100 us

	std::double_t Vfdi = 0.76;
	std::double_t SlopeDi = 17.77;
	std::double_t Vfce = 0.76;
	std::double_t SlopeIc = 20.0;

	//Brushed DC Motor PITTMAN DC054B-7 54mm
	std::double_t b = 0.0000135;
	std::double_t Kb = 0.02627; //in Vs/rad
	std::double_t J = 0.000047;
	std::double_t R = 0.24;
	std::double_t L = 0.31e-3;

	std::double_t Kt = 0.0424;

	motor.setParameters(ts, b, Kb ,J , R, L, Kt,Vfdi, SlopeDi, Vfce,SlopeIc );

	PIDController piVel;
	PIDController piTq;

	std::double_t kpT = 9.5;
	std::double_t TiT = 0.001;
	std::double_t kiT = kpT/TiT;
	std::double_t kdT = 0.00;
	std::double_t kbT = 1.00;

	std::double_t kp = 0.05;
	std::double_t Ti = 0.2;
	//std::double_t ki = kp / Ti;
	std::double_t ki = 0.01;
	std::double_t kd = 0.00;
	std::double_t kb = 1.00;

	std::double_t upSaturationTq = 0.76; //max 0.76 Nm
	std::double_t upSaturationU = 16.00; //max 16 V
	
	piTq.setParameters(kpT, kiT, kdT, kbT, tsc, upSaturationU);
		
	piVel.setParameters(kp, ki, kd, kb, tsc, upSaturationTq);

	double error = 0.0;
	double errorTq = 0.0;

	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestDCMotorPWM.dat";

	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());


	auto refVelShPtr = tracer.addSignal<std::double_t>("refVel", BaseSignal::SignalType::Double);
	auto motorVelShPtr = tracer.addSignal<std::double_t>("Vel", BaseSignal::SignalType::Double);
	auto refTqShPtr = tracer.addSignal<std::double_t>("refTq", BaseSignal::SignalType::Double);
	auto motorTqShPtr = tracer.addSignal<std::double_t>("Tq", BaseSignal::SignalType::Double);

	tracer.writeHeader();

	refVelShPtr->set(60.0 * 6.28); ///1000 rpm

	std::double_t dcBusVoltage = upSaturationU; //const at the moment

	for (std::uint32_t i = 0U; i < 1000U; i++)
	{
		error = refVelShPtr->get() - motorVelShPtr->get();		
		double tqref = piVel.process(error);				
		errorTq = tqref - motor.getTorque();
		double val = piTq.process(errorTq);
		refTqShPtr->set(tqref);

		if (i < 400)
		{
			refVelShPtr->set(60.0 * 6.28); ///1000 rpm cw
		}
		else
		{
			refVelShPtr->set(-60.0 * 6.28); ///1000 rpm ccw
		}

		//calculate duty cycle
		std::double_t duty_cycle = val / dcBusVoltage;
		//ON in the midle of the cycle
		std::int32_t count = (std::uint32_t) (duty_cycle * 100);
		std::uint32_t start = 50 - abs(count / 2);
		std::uint32_t end = 50 + abs(count / 2);
	
		for (std::uint32_t j = 0; j < 100; j++)
		{
			if (j <= start || j >= end)
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
			
			motor.process();
			motorVelShPtr->set(motor.getVelocity());
			motorTqShPtr->set(motor.getTorque());
			tracer.trace();
		}		
	}
	std::double_t i = motor.getCurrent();
	std::double_t w = motor.getVelocity();


}
*/