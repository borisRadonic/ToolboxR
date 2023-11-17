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

#include "PMSMotor.h"
#include "PMSMPICurrentController.h"
#include "WaveFormTracer.h"


using namespace CntrlLibrary;
using namespace Models;

using std::experimental::filesystem::path;

using namespace DiscreteTime;
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
	motor.setParameters(ts, pole_pairs, b, Kemf, J, Rs, Lq, Ld, Kt, Tf);

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
		tracer.trace();
	}
}