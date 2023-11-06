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

	QuinticPolyTrajectory traj( i_pos,
								i_vel,
								i_accel,
								f_pos,
								f_vel,
								f_accel,								
								m_accel,
								m_vel );
		
	f_time = traj.calculateMinTime(0.5,0.5);
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

//#ifdef TEST_7453457849
TEST(TestCaseJerkLimitedTrajectory, TestBasicJerkLimitedTrajectory)
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
	double i_vel = 100.00;
	double i_accel = 0.00;
	double f_pos = 2.00;
	double f_vel = 0.00;
	double f_accel = 0.00;
	double f_time = 0.10;
	

	traj.setParameters(20000.0, 1000.0, 300.00, 0.000001); // Max jerk, acceleration, velocity and processing time 
	traj.setInitialValues( i_pos, i_vel, i_accel);
	traj.setFinalValues(f_pos, f_vel, f_accel);  // Target position, Target velocity, Target acceleration

	traj.prepare(f_time);

	double pos, vel, accel;
	for (double t = 0.000; t <= f_time; t = t + 0.001)
	{
		traj.process(t, pos, vel, accel);
		acceleration->set(accel);
		velocity->set(vel);
		position->set(pos);
		tracer.trace();
	}

}
//#endif
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

TEST(TestCasePMSM, TestPMSM)
{
	PMSMotor motor;
	std::double_t ts = 0.0001; //100 us
	//PMSM parameters
	std::uint16_t pole_pairs = 1; //number of pole pairs
	std::double_t b = 0.00047; //Combined friction of rotor and load
	std::double_t Kemf = 0.1327; //in Vs/rad
	std::double_t J = 0.0047; //Combined moment of inertia of rotor and load
	std::double_t Rs = 0.74; //Stator resistance in [Ohm]
	std::double_t Lq = 7.84e-3; //Inductance of stator in dq frame (q part) in [H]
	std::double_t Ld = 7.14e-3; //Inductance of stator in dq frame (d part) in [H]
	std::double_t Kt = 0.83; //Torque constant in [A/Nm]
	std::double_t Tf = 0.0135; //Static friction torque in [Nm]
	std::double_t dcBusVoltage = 320.00; //const at the moment

	std::double_t peakTq = 14.3; //Peak torque in Nm ( Base speed 2000 rpm )
	//Nominal torque 7.16 Nm, Nominal speed 2000 rpm, Maximum current Irms 20.00 A
	//Maximum winding voltage VDC 360


	motor.setParameters( ts, pole_pairs, b, Kemf, J, Rs, Lq, Ld, Kt, Tf);

	PIDController piVel;
	PMSMPICurrentController tqController;
	Filters::IIRFirstOrderFilter velPreFlt;
		
	std::double_t kp = 0.49;
	std::double_t Ti = 0.3;
	std::double_t ki = kp/Ti;
	std::double_t kd = 0.00;
	std::double_t kb = 1.00;

	//Velocity First order IIR Filter coefficients
	std::double_t Wc_vel = 6.00; //rad/sec
	std::double_t alpha_vel = exp( -Wc_vel * ts); //rad/sec
	std::double_t iirVelpre_a1 = -alpha_vel;
	std::double_t iirVelpre_b0 = 1 - alpha_vel;
	std::double_t iirVelpre_b1 = 0.00; //first orderer
	velPreFlt.setParameters(iirVelpre_a1, iirVelpre_b0, iirVelpre_b1);

	std::double_t upSaturationTq = 12.0; //max 15 Nm
	

	//max bandwith is dependant from motor electrical constant
	std::double_t tau_e = Lq / Rs;
	
	//we do not use velocity filter in torque controller at the moment
	std::double_t iirVelFlt_a1 = 0.00;
	std::double_t iirVelFlt_b0 = 0.00;
	std::double_t iirVelFlt_b1 = 0.00; //first order filter

	std::double_t W_c_tq = 1000; /// frequency of the torque/current controller prefilters in [rad/sec]
	std::double_t alpha_q = exp(-W_c_tq * ts); //rad/sec
	std::double_t iirPreFlt_a1 = -alpha_q;
	std::double_t iirPreFlt_b0 = 1- alpha_q;
	std::double_t iirPreFlt_b1 = 0.00;
	
	std::double_t kp_q = 15.0;
	std::double_t ki_q = kp_q/0.001;
	std::double_t kd_q = 0.00;
	std::double_t kb_q = 1.00;
	std::double_t upSat_q = 300.00;
	std::double_t kp_d = kp_q;
	std::double_t ki_d = ki_q;
	std::double_t kd_d = kd_q;
	std::double_t kb_d = kb_q;
	std::double_t upSat_d = upSat_q;
	
	piVel.setParameters(kp, ki, kd, kb, ts, upSaturationTq);

	tqController.setParameters( iirPreFlt_a1,
								iirPreFlt_b0,
								iirPreFlt_b1,
								iirVelFlt_a1,
								iirVelFlt_b0,
								iirVelFlt_b1,
								kp_q,
								ki_q,
								kd_q,
								kb_q,
								upSat_q,
								kp_d,
								ki_d,
								kd_d,
								kb_d,
								upSat_d,
								Ld,
								Lq,
								Kemf,
								pole_pairs,
								ts);

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestPMSM.dat";

	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto motorIdShPtr = tracer.addSignal<std::double_t>("Id", BaseSignal::SignalType::Double);
	auto refTqShPtr = tracer.addSignal<std::double_t>("refIq", BaseSignal::SignalType::Double);
	auto motorIqShPtr = tracer.addSignal<std::double_t>("Iq", BaseSignal::SignalType::Double);
	auto refVelShPtr = tracer.addSignal<std::double_t>("refVel", BaseSignal::SignalType::Double);
	auto motorVelShPtr = tracer.addSignal<std::double_t>("Vel", BaseSignal::SignalType::Double);

	tracer.writeHeader();
	
	for (std::uint32_t i = 0U; i < 60000U; i++)
	{

		double error = refVelShPtr->get() - motorVelShPtr->get();
		double tqref = piVel.process(error);		
	

		if (i < 30000)
		{
			
			refVelShPtr->set(velPreFlt.process(60.0 * 6.28)); ///1000 rpm cw
		}
		else
		{
			refVelShPtr->set(velPreFlt.process(-60.0 * 6.28)); ///1000 rpm ccw
		}

			
		refTqShPtr->set(tqref / Kt);
		tqController.process(tqref / Kt, 0.0, motor.getIq() , motor.getId(), motor.getPos() );
		if (i > 50000 )
		{
			motor.setInputs(tqController.getUq(), tqController.getUd(), 0.00);
		}
		motor.setInputs(tqController.getUq(), tqController.getUd(), 0.00);
		motor.process();

		motorIdShPtr->set(motor.getId());
		motorVelShPtr->set(motor.getVel());
		motorIqShPtr->set(motor.getIq());
		tracer.trace();
	}
}