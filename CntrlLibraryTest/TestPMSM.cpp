/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/
#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <vector>

#include "StateSpace.h"

#include "PMSMotor.h"
#include "PMSMotorWithFlexibleCoupling.h"
#include "PIDController.h"
#include "StringUtil.h"

#include "PMSMotor.h"
#include "PMSMPICurrentController.h"
#include "PMSMPositionController.h"
#include "FrictionModelCV.h"
#include "JerkLimitedTrajectory.h"
#include "WaveFormTracer.h"


using namespace CntrlLibrary;
using namespace Models;

using std::experimental::filesystem::path;

using namespace DiscreteTime;
TEST(TestCasePMSM, TestPMSMVelCurrentCntr)
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

	std::double_t Tsf = 0.02; //static friction
	//Nominal torque 7.16 Nm, Nominal speed 2000 rpm, Maximum current Irms 20.00 A
	//Maximum winding voltage VDC 360
	motor.setParameters(ts, pole_pairs, b, Kemf, J, Rs, Lq, Ld, Kt, Tsf);

	PIDController piVel;
	PMSMPICurrentController tqController;
	Filters::IIRFirstOrderFilter velPreFlt;

	std::double_t kp = 0.49;
	std::double_t Ti = 0.3;
	std::double_t ki = kp / Ti;
	std::double_t kd = 0.00;
	std::double_t kb = 1.00;

	//Velocity First order IIR Filter coefficients
	std::double_t Wc_vel = 6.00; //rad/sec
	std::double_t alpha_vel = exp(-Wc_vel * ts); //rad/sec
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
	std::double_t iirPreFlt_b0 = 1 - alpha_q;
	std::double_t iirPreFlt_b1 = 0.00;

	std::double_t kp_q = 15.0;
	std::double_t ki_q = kp_q / 0.001;
	std::double_t kd_q = 0.00;
	std::double_t kb_q = 1.00;
	std::double_t upSat_q = 300.00;
	std::double_t kp_d = kp_q;
	std::double_t ki_d = ki_q;
	std::double_t kd_d = kd_q;
	std::double_t kb_d = kb_q;
	std::double_t upSat_d = upSat_q;

	piVel.setParameters(kp, ki, kd, kb, ts, upSaturationTq);

	tqController.setParameters(iirPreFlt_a1,
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
	auto cntrValphaPtr = tracer.addSignal<std::double_t>("V_alpha", BaseSignal::SignalType::Double);
	auto cntrVbetaPtr = tracer.addSignal<std::double_t>("V_beta", BaseSignal::SignalType::Double);
		

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
		tqController.process(tqref / Kt, 0.0, motor.getIq(), motor.getId(), motor.getPos());
		if (i > 50000)
		{
			motor.setInputs(tqController.getUq(), tqController.getUd(), 0.00);
		}
		motor.setInputs(tqController.getUq(), tqController.getUd(), 0.00);
		motor.process();
		
		motorIdShPtr->set(motor.getId());
		motorVelShPtr->set(motor.getVel());
		motorIqShPtr->set(motor.getIq());
		cntrValphaPtr->set(tqController.get_v_alpha());
		cntrVbetaPtr->set(tqController.get_v_beta());
		tracer.trace();
	}
}


TEST(TestCasePMSM, TestPMSMPosController)
{
	using namespace TrajectoryGeneration;

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
	std::double_t Tsf = 0.01; //static friction
	std::double_t dcBusVoltage = 320.00; //const at the moment

	std::double_t peakTq = 14.3; //Peak torque in Nm ( Base speed 2000 rpm )
	//Nominal torque 7.16 Nm, Nominal speed 2000 rpm, Maximum current Irms 20.00 A
	//Maximum winding voltage VDC 360
	motor.setParameters(ts, pole_pairs, b, Kemf, J, Rs, Lq, Ld, Kt, Tsf);
		
	std::double_t posLimitPos = 10000.00;
	std::double_t posLimitNeg = -10000.00;

	std::double_t max_vel = 300.00; //velocity limit
	std::double_t max_torque = 12.00; //torque limit
	
	std::double_t ff_vel_gain = 1.00; //in this case 1.00
	std::double_t ff_accel_gain = 1.00 * J; // to convert acceleration to Feed Forward Torque


	std::double_t  max_jerk = 5000.0;

	std::double_t  max_accel = 200.0;

	std::double_t ff_jerk_gain = 0.2 * max_torque / max_jerk;
	ff_jerk_gain = 0.00;
	
	std::double_t pos_kp = 45.05;

	std::double_t vel_kp = 0.46;
	std::double_t vel_ki = 5.3;

	std::double_t kd = 0.00;
	std::double_t kb = 1.00;

	std::double_t Wc_vel = 3200; //rad/sec --test
	


	PMSMPositionController positionController;
	positionController.setSamplinkPeriod(0.0001);
	positionController.setLimitParameters( posLimitNeg, posLimitPos, max_vel, max_torque);
	positionController.setFeedForwardParameters(ff_vel_gain, ff_accel_gain, ff_jerk_gain);
	positionController.setPosVelControllerParameters(pos_kp, vel_kp, vel_ki, Wc_vel, Kt);
	//if we do not use notch filter, we simply do not set filter parameters
	//positionController.setNotchFilterParameters(...); 
	
	
	//max bandwith is dependant from motor electrical constant
	std::double_t tau_e = Lq / Rs;

	//we do not use velocity filter in torque controller at the moment
	std::double_t iirVelFlt_a1 = 0.00;
	std::double_t iirVelFlt_b0 = 0.00;
	std::double_t iirVelFlt_b1 = 0.00; //first order filter

	std::double_t W_c_tq = 1000; /// frequency of the torque/current controller prefilters in [rad/sec]
	std::double_t alpha_q = exp(-W_c_tq * ts); //rad/sec
	std::double_t iirPreFlt_a1 = -alpha_q;
	std::double_t iirPreFlt_b0 = 1 - alpha_q;
	std::double_t iirPreFlt_b1 = 0.00;

	std::double_t kp_q = 20.0;
	std::double_t ki_q = kp_q / 0.001;
	std::double_t kd_q = 0.00;
	std::double_t kb_q = 1.00;
	std::double_t upSat_q = 300.00;
	std::double_t kp_d = kp_q;
	std::double_t ki_d = ki_q;
	std::double_t kd_d = kd_q;
	std::double_t kb_d = kb_q;
	std::double_t upSat_d = upSat_q;

	positionController.setCurrentControllerParameters(	iirPreFlt_a1,
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
														pole_pairs);

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestPMSMPos.dat";

	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto motorIdPtr = tracer.addSignal<std::double_t>("Id", BaseSignal::SignalType::Double);
	auto refTqPtr = tracer.addSignal<std::double_t>("refIq", BaseSignal::SignalType::Double);
	auto motorIqPtr = tracer.addSignal<std::double_t>("Iq", BaseSignal::SignalType::Double);
	auto refVelPtr = tracer.addSignal<std::double_t>("refVel", BaseSignal::SignalType::Double);
	auto motorVelPtr = tracer.addSignal<std::double_t>("Vel", BaseSignal::SignalType::Double);
	auto cntrValphaPtr = tracer.addSignal<std::double_t>("V_alpha", BaseSignal::SignalType::Double);
	auto cntrVbetaPtr = tracer.addSignal<std::double_t>("V_beta", BaseSignal::SignalType::Double);	
	auto cntrRefPosPtr = tracer.addSignal<std::double_t>("refPos", BaseSignal::SignalType::Double);
	auto cntrSenPosPtr = tracer.addSignal<std::double_t>("isPos", BaseSignal::SignalType::Double);	
	auto cntrRefAccelPtr = tracer.addSignal<std::double_t>("RefAccel", BaseSignal::SignalType::Double);
	auto cntrJerkPtr = tracer.addSignal<std::double_t>("Jerk", BaseSignal::SignalType::Double);

	tracer.writeHeader();
	std::double_t ref_position(0.00);
	std::double_t iq_s(0.00);
	std::double_t id_s(0.00);
	std::double_t s_position(0.00);
	std::double_t s_velocity(0.00);
	std::double_t ff_acceleration(0.00);
	std::double_t ff_velocity(0.00);
	std::double_t ff_torque_offset = 0.00; //no offset
	std::double_t ff_torque_compensations = 0.00; //no compensation in this Test case
	std::double_t ff_jerk = 0.00;

	JerkLimitedTrajectory traj;

	std::double_t i_pos = 0.00;
	std::double_t i_vel = 0.00;
	std::double_t i_accel = 0.00;
	std::double_t f_pos = 200.00;
	std::double_t f_vel = 0.00;
	std::double_t f_accel = 0.00;
	std::double_t f_time = 10.0;
	std::double_t fric_tq = 0.00;

	traj.setParameters(max_jerk, max_accel, 100.00, 0.00001, 0.001, 0.1); // Max jerk, Max. acceleration, Max. velocity, max. pos. error, max vel. error and max. accel. error
	traj.setInitialValues(i_pos, i_vel, i_accel);
	traj.setFinalValues(f_pos, f_vel, f_accel);  // Target position, Target velocity, Target acceleration

	traj.prepare(f_time);

	FrictionModelCSV friction;
	friction.setParameters(ts, b, Tsf, J);



	for (double t = 0.00001; t <= f_time + 0.00001; t = t + ts)
	{

		if (t > (f_time- 0.0004) && t < (f_time + 0.00001))
		{
			int a = 0;
			a++;
		}
		traj.process(t, ref_position, ff_velocity, ff_acceleration, ff_jerk);
 		
		/*compensate friction*/
		friction.setInputs(motor.getVel(), motor.getIq() / Kt, ff_acceleration);
		friction.process();
		fric_tq = friction.getFrictionTorque();
		fric_tq = 0.00;

		positionController.process(ref_position,
			motor.getIq(),
			motor.getId(),
			motor.getPos(),
			motor.getVel(),
			ff_acceleration,
			ff_velocity,
			ff_torque_offset,
			ff_torque_compensations + fric_tq, ff_jerk);
		
		refTqPtr->set(positionController.get_refTq() / Kt);
		motor.setInputs(positionController.getUq(), positionController.getUd(), 0.00);
		motor.process();

		motorIdPtr->set(motor.getId());
		motorVelPtr->set(motor.getVel());
		motorIqPtr->set(motor.getIq());
		cntrValphaPtr->set(positionController.get_v_alpha());
		cntrVbetaPtr->set(positionController.get_v_beta());
		cntrRefAccelPtr->set(ff_acceleration);
		refVelPtr->set(ff_velocity);
		cntrRefPosPtr->set(ref_position);
		cntrSenPosPtr->set(motor.getPos());
		cntrJerkPtr->set(ff_jerk);
		tracer.trace();
	}
}

TEST(TestCasePMSM, TestFlexCouplingFreqResponse)
{
	PMSMotorWithFlexibleCoupling motor;
	std::double_t ts = 0.0001; //100 us
	//PMSM parameters
	std::uint16_t pole_pairs = 1; //number of pole pairs
	std::double_t b = 0.00047; //Combined friction of rotor and load
	std::double_t Kemf = 0.1327; //in Vs/rad
	std::double_t J = 0.007; //Combined moment of inertia of rotor and load	
	std::double_t Rs = 0.74; //Stator resistance in [Ohm]
	std::double_t Lq = 7.84e-3; //Inductance of stator in dq frame (q part) in [H]
	std::double_t Ld = 7.14e-3; //Inductance of stator in dq frame (d part) in [H]
	std::double_t Kt = 0.83; //Torque constant in [A/Nm]
	std::double_t Tf = 0.0135; //Static friction torque in [Nm]
	std::double_t dcBusVoltage = 320.00; //const at the moment

	std::double_t peakTq = 14.3; //Peak torque in Nm ( Base speed 2000 rpm )

	std::double_t Tsf = 0.02; //static friction
	//Nominal torque 7.16 Nm, Nominal speed 2000 rpm, Maximum current Irms 20.00 A
	//Maximum winding voltage VDC 360
	motor.setParameters(ts, pole_pairs, b, Kemf, J, Rs, Lq, Ld, Kt, Tsf);
	motor.setLoadAndCouplingParameters(0.01, 4000.00);

	PMSMPICurrentController tqController;
	Filters::IIRFirstOrderFilter velPreFlt;

	std::double_t kp = 0.49;
	std::double_t Ti = 0.3;
	std::double_t ki = kp / Ti;
	std::double_t kd = 0.00;
	std::double_t kb = 1.00;

	//Velocity First order IIR Filter coefficients
	std::double_t Wc_vel = 6.00; //rad/sec
	std::double_t alpha_vel = exp(-Wc_vel * ts); //rad/sec
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
	std::double_t iirPreFlt_b0 = 1 - alpha_q;
	std::double_t iirPreFlt_b1 = 0.00;

	std::double_t kp_q = 15.0;
	std::double_t ki_q = kp_q / 0.001;
	std::double_t kd_q = 0.00;
	std::double_t kb_q = 1.00;
	std::double_t upSat_q = 300.00;
	std::double_t kp_d = kp_q;
	std::double_t ki_d = ki_q;
	std::double_t kd_d = kd_q;
	std::double_t kb_d = kb_q;
	std::double_t upSat_d = upSat_q;

	tqController.setParameters(iirPreFlt_a1,
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

	//for Reequency Ruse only torque controller and the plant

}