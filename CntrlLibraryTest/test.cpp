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
#include "FisFileExport.h"

#include "WaveFormTracer.h"

#include "DCMotor.h"
#include "FrictionModelCV.h"
#include "PIDController.h"

#include "StringUtil.h"

#include "HexicPolynomial.h"
#include "HepticPolynomial.h"
#include "SimplelHepticTrajectory.h"
#include "QuinticBezierCurve.h"
#include "ConstFunction.h"
#include "PathSegment.h"

#include <Eigen/Dense>

using namespace CntrlLibrary;
using namespace Models;



using std::experimental::filesystem::path;

using namespace DiscreteTime;


TEST(TestQuanticBezierCurve, TestQuanticBezierCurve1)
{
	using namespace Math;
	using namespace Bezier;
	using namespace TrajectoryGeneration;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestBezierCurve.dat";
	std::string fileName2 = strpath + "test/TestBezierCurve2.dat";

	double ts = 0.0001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	WaveFormTracer tracer2(fileName2, ts);
	EXPECT_TRUE(tracer2.open());

	auto trS = tracer.addSignal<std::double_t>("x", BaseSignal::SignalType::Double);
	auto trSI = tracer.addSignal<std::double_t>("xI", BaseSignal::SignalType::Double);
	auto trV = tracer.addSignal<std::double_t>("v", BaseSignal::SignalType::Double);
	auto trVI = tracer.addSignal<std::double_t>("vI", BaseSignal::SignalType::Double);
	auto trA = tracer.addSignal<std::double_t>("a", BaseSignal::SignalType::Double);


	auto trErrV = tracer2.addSignal<std::double_t>("Error v", BaseSignal::SignalType::Double);
	auto trErrS = tracer2.addSignal<std::double_t>("Error x", BaseSignal::SignalType::Double);
	

	double a = 0.00;
	double v = 0.00;
	double s = 0.00;
	

	double tf = 0.1;

	double i_pos = 0.00;
	double i_vel = 0.00;
	double i_accel = 0.00;
	double f_pos = 100.00;
	double f_vel = 0.00;
	double f_accel = 0.00;

	double v_max = 300.00;
	double a_max = 600.00;
	double j_max = 16000.00;
	
	
	double travel_distance = f_pos - i_pos;
	
	double Td = 0.00;
	double Tv = 0.00;


	double Ac = 0.00;
	double Dc = 0.00;
	double Vc = 0.00;

	double sign = 1.00;
	if (travel_distance <= 0.00)
	{
		sign = -1.00;
	}
	Ac = sign * a_max;
	Dc = -sign * a_max;

	/*effect of jerk phases*/
	double Tjv = abs(Ac / j_max);
	double Tjd = abs((Dc - i_accel) / j_max);

	/*minimum travel time will never be less then times required for jerk phases (even if all jerk phases do not exist)*/
	double tphAp = abs(Ac - i_accel) / j_max; //max. duration of + jerk phase
	double tphAm = abs(Ac) / j_max; //max. duration of - jerk phase
	double tphDp = abs(Dc) / j_max;  //max. duration of + jerk phase for deacceleration
	double tphDm = (abs(Dc) - f_accel) / j_max; //max. duration of - jerk phase for deacceleration

	//during the jerk phase average acceleration is 50% of max ( for our Bazier sigmoid function )
	double aproxJerkDistAp = abs(0.25 * (Ac - i_accel) * tphAp * tphAp);
	double aproxJerkDistAm = abs(0.25 * (Ac) * tphAm * tphAm);
	double aproxJerkDistDp = abs(0.25 * (Dc - i_accel) * tphDp * tphDp);
	double aproxJerkDistDm = abs(0.25 * (abs(Dc) - f_accel) * tphDm * tphDm);
	
	/*during A- and D+ velocity ist nearly Vc (we do not know Vc), but we take max_v*/
	aproxJerkDistAp += abs( tphAp * i_vel);
	aproxJerkDistAm += abs( tphAm * v_max );
	aproxJerkDistDp += abs(tphDp * v_max);
	aproxJerkDistDm += abs(tphDm * f_vel);


	double aproxJerkDistance = aproxJerkDistAp + aproxJerkDistAm + aproxJerkDistDp + aproxJerkDistDm;

	double min_max_vel_distance = abs((v_max * v_max - (i_vel * i_vel)) / (2.00 * a_max)
		+ v_max * v_max / (2.00 * a_max));



	min_max_vel_distance += aproxJerkDistance;

	//the condition for 'triangular profile'
	EXPECT_TRUE( abs(travel_distance) < min_max_vel_distance );




	Tv = 0.00;
	Vc = sign * sqrt((2.00 * a_max * a_max * (travel_distance - aproxJerkDistance) - a_max * i_vel * i_vel) / (a_max + a_max));
		
	double Ta = abs( (Vc - i_vel) / Ac);

	Td = abs( (Vc-f_vel) / Dc );
						


	/*create 4 Bazier curves:
	* 
	* A+ Acceleration	+
	* A- Acceleration	-
	* D+ Deacceleration +
	* D- Deacceleration -
	
	7 phases:  A+, const Ac, A-, Const Vc, D+, Const -Dc, D-,
	

	Times: tphAp, tphAc, tphAm, tphVc, tphDp, tphDc, tphDm, 
	*/

	

	//scale for total time
	double time_scale_factor1 = 1.00 / tphAp;
	double time_scale_factor2 = 1.00 / tphAm;

	double P0 = i_accel;  //start Acceleration
	double P1 = i_accel;  //it influence the initial rise phase
	double P2 = i_accel;  //it shapes the middle part of the curve
	double P3 = Ac;  //it shapes the middle part of the curve
	double P5 = Ac; // it influence the final  phase
	double P4 = Ac; // it influence the final  phase


	std::shared_ptr<QuinticBezierCurve> curveAplus = std::make_shared<QuinticBezierCurve>();
	curveAplus->setParams(P0, P1, P2, P3, P4, P5);
	

	//test
	P0 = 0;
	P1 = 0;
	P2 = 0;
	P3 = 1.00;
	P4 = 1.00; // /it influence the final  phase
	P5 = 1.00; //stop acceleration

	QuinticBezierCurve test1;
	test1.setParams(P0, P1, P2, P3, P4, P5);
	double in0 = test1.firstIntegral(0.00);
	double in1 = test1.firstIntegral(1.00);
	double myBConst = 0.6666666666666673;
	double myBconst2 = 1.00 - myBConst;

	P0 = Ac; //start Acceleration
	P1 = Ac; //it influence the initial phase
	P2 = Ac; //it shapes the middle part of the curve
	P3 = 0.00; //it shapes the middle part of the curve
	P4 = 0.00; // /it influence the final  phase
	P5 = 0.00; //stop acceleration
	

	std::shared_ptr<QuinticBezierCurve> curveAminus = std::make_shared<QuinticBezierCurve>();
	curveAminus->setParams(P0, P1, P2, P3, P4, P5);

	double ConstIntAm0 = curveAminus->firstIntegral(0.00);
	
	PathSegment plusAccelPath(0.00, 1.00, 0.00, i_vel, i_pos, curveAplus);
		
	double ConstIntAp0 = plusAccelPath.getFirstIntegralAtStart();

	double velocityAp = (curveAplus->firstIntegral(1.00) - ConstIntAp0 ) * tphAp;
	double velocityAm = (curveAminus->firstIntegral(1.00) - ConstIntAm0) * tphAm;

	//calculate time of constant acceleration to reach Vc
	double tphAc = (Vc - (velocityAp + velocityAm)) / Ac;

		
	//velocity part ( velocity is Vc during the whole period)
	
	double velAc = velocityAp + Ac * (tphAc);
		
	double constantI2 = curveAminus->secondIntegral(0.00);

	double valAm = constantI2 - curveAminus->secondIntegral(1.0);
	

	double distanceAc = abs(0.50 * Ac * tphAc * tphAc) + (velocityAp) * tphAc;

	tphAc += tphAp;
	tphAm += tphAc;
		

	Integrator integral;
	integral.setParameters(IntegratorMethod::BackwardEuler, 0.0001, 1.00);
	integral.setInitialConditions(0.00);

	Integrator integral2;
	integral2.setParameters(IntegratorMethod::BackwardEuler, 0.0001, 1.00);
	integral2.setInitialConditions(i_pos);
		

	//calculate distance Ap

	double distanceAp = tphAp * tphAp * (curveAplus->secondIntegral(1.00) - curveAplus->secondIntegral(0.00));

	

	velocityAp = plusAccelPath.getVelocity(1.00, tphAp);
	
	double pos1 = 0.00;
	double pos2 = 0.00;

	for (double t = 0.00; t < (tphAp + 0.000001); t = t + 0.0001)
	{
		double tau = t * time_scale_factor1;
		
		a = plusAccelPath.getAccel(tau);
		v = plusAccelPath.getVelocity(tau, tphAp);
		s = plusAccelPath.getPosition(tau, tphAp, tphAp * tphAp);
		pos1 = s;
		if (pos2 > pos1)
		{
			int a = 0;
			a++;
		}
		pos2 = pos1;
		trS->set(s);
		trA->set(a);
		trV->set(v);
		
		double intA = integral.process(a);
		trVI->set(intA);
		trSI->set(integral2.process(v));
		//trSI->set(integral2.process(intA));
		
		trErrV->set(trVI->get() - v);
		trErrS->set(trSI->get() - s);
		tracer.trace();
		tracer2.trace();
	}
	integral.reset();
	integral.setInitialConditions(velocityAp);

	integral2.reset();
	integral2.setInitialConditions(distanceAp);
	
	/*const acceleration*/
	
	std::shared_ptr<ConstFunction> mathFuncConst = std::make_shared<ConstFunction>(Ac);
	PathSegment constAccelPath(tphAp, tphAc, 0.00, velocityAp, distanceAp, mathFuncConst);

	double testdiff = constAccelPath.getPosition(tphAc);


	//x1 = 0.00;
	for (double t = (tphAp ); t < (tphAc + 0.000001); t = t + 0.0001)
	{
		a = constAccelPath.getAccel(t);
		v = constAccelPath.getVelocity(t);
		s = constAccelPath.getPosition(t);
				
		trS->set(s);
		trA->set(a);
		trV->set(v); 
	
		double intA = integral.process(a);
		trVI->set(intA);
		trSI->set(integral2.process(v));
		//trSI->set(integral2.process(intA));

		trErrV->set(trVI->get() - v);
		trErrS->set(trSI->get() - s);

		tracer.trace();
		tracer2.trace();
	}
	integral.reset();
	integral.setInitialConditions(v);
	
	integral2.reset();
	integral2.setInitialConditions(distanceAc + distanceAp);


	PathSegment minusAccelPath(0.00, 1.00, 0.00, velAc, distanceAc + distanceAp, curveAminus);

	double distanceAm = minusAccelPath.getPosition(1.00, tphAm - tphAc, (tphAm - tphAc) * (tphAm - tphAc));

	for (double t = (tphAc ); t < (tphAm +0.000001); t = t + 0.0001)
	{
		double delta_t = (t - tphAc);
		double tau = delta_t * time_scale_factor2;
				
		if (tau <= 1.00)
		{
			a = minusAccelPath.getAccel(tau);
			v = minusAccelPath.getVelocity(tau, tphAm - tphAc, true );
			s = minusAccelPath.getPosition(tau, tphAm - tphAc, (tphAm - tphAc) * (tphAm - tphAc) );
		} 

		trS->set(s);
		trA->set(a);
		trV->set(v);
		
		double intA = integral.process(a);
		trVI->set(intA);
		trSI->set(integral2.process(v));
		//trSI->set(integral2.process(intA));

		trErrV->set(trVI->get() - v);
		trErrS->set(trSI->get() - s);
		tracer.trace();
		tracer2.trace();
	}
	//integral.reset();
	//integral.setInitialConditions(v);
	
	//integral2.reset();
	//integral2.setInitialConditions(s);


	double deltaD = Dc - f_accel;

	//standard d- for path generation

	P0 = deltaD;
	P1 = deltaD;
	P2 = deltaD;
	P3 = 0.00;
	P4 = 0.00;
	P5 = 0.00;

	std::shared_ptr<QuinticBezierCurve> funcDminus = std::make_shared<QuinticBezierCurve>();
	funcDminus->setParams(P0, P1, P2, P3, P4, P5);


	P0 = 0;
	P1 = 0;
	P2 = 0;
	P3 = abs(Dc);
	P4 = abs(Dc);
	P5 = abs(Dc);

	//we can change the angle of view for computations

	std::shared_ptr<QuinticBezierCurve> curveDplus = std::make_shared<QuinticBezierCurve>();

	curveDplus->setParams(P0, P1, P2, P3, P4, P5);
	double ConstIntDp0 = tphDp * curveDplus->firstIntegral(0.00);
	
	
	P0 = 0.00;
	P1 = 0.00;
	P2 = 0.00;
	P3 = abs(deltaD);
	P4 = abs(deltaD);
	P5 = abs(deltaD);
		
	std::shared_ptr<QuinticBezierCurve> curveDminus = std::make_shared<QuinticBezierCurve>();
	curveDminus->setParams(P0, P1, P2, P3, P4, P5);

	double ConstIntDm0 = curveDminus->firstIntegral(0.00);

	double velocityDp = (curveDplus->firstIntegral(1.00) - ConstIntDp0) * tphDp;
	double velocityDm = (curveDminus->firstIntegral(1.00) - ConstIntDm0) * tphDm;

	//correct Vc
	Vc = velAc + (tphAm - tphAc) * (curveAminus->firstIntegral(1.00) - ConstIntAm0);


	
	double posAtStartDp = i_pos + distanceAm;
	//calculate distances

	

	double ConstSecIntDp0 = curveDplus->secondIntegral(0.00);
	double distanceDp = abs(tphDp * tphDp * (ConstSecIntDp0 - curveDplus->secondIntegral(1.00)));

	double velDp = Vc - abs(velocityDp);
	
	


	//calculate time of constant deacceleration to reach final velocity
	double tphDc = abs((Vc - (abs(velocityDp) + abs(velocityDm)) - f_vel) / Dc);
	double distanceDc = (velDp)*tphDc - abs(0.50 * Dc * tphDc * tphDc);
		
	
	distanceDp = Vc * tphDp - distanceDp;
	double distanceDm = abs(tphDm * tphDm * (curveDminus->secondIntegral(1.00) - curveDminus->secondIntegral(0.00)) );
	

				
	double dis = i_pos + distanceAm + distanceDp + distanceDc + distanceDm;

	//calculate constant velocity distance
	double distToDo = f_pos - dis;

	EXPECT_TRUE(distToDo >= 0.00);
		

	
	double tphVc = abs( distToDo / Vc );
		
	/*const velocity Vc*/
	double tAtVc = tphAm + tphVc;

	tphDp = tAtVc + tphDp;

	tphDc += tphDp;

	tphDm += tphDc;

	//phase D+
	PathSegment deaccelPlusPath(0.00, 1.00, 0.00, Vc, posAtStartDp + abs(distToDo), curveDminus);
		

	std::shared_ptr<ConstFunction> mathFuncConstAccel0 = std::make_shared<ConstFunction>(0.00);
	PathSegment constAccel0Path(tphAm, tAtVc, 0.00, Vc, posAtStartDp, mathFuncConst);

	for (double t = (tphAm+0.0001); t < (tAtVc + 0.000001); t = t + 0.0001)
	{
		s = constAccel0Path.getPosition(t, 1.00, 0.00);

		trS->set(s);
		trA->set(a);
		trV->set(v);

		double intA = integral.process(a);
		trVI->set(intA);
		trSI->set(integral2.process(v));
		//trSI->set(integral2.process(intA));


		trErrV->set(trVI->get() - v);
		trErrS->set(trSI->get() - s);

		tracer.trace();
		tracer2.trace();
	}
	integral.reset();
	integral.setInitialConditions(v);
	
	integral2.reset();
	integral2.setInitialConditions(s);

	

	for (double t = (tAtVc + 0.0001); t <= (tphDp + 0.000001); t = t + 0.0001)
	{
		double delta_t = (t - tAtVc);
		double tau = delta_t * time_scale_factor2;

		double tp = tphDp - tAtVc;
		if (tau <= 1.00)
		{
			a = -deaccelPlusPath.getAccel(tau);
			v = deaccelPlusPath.getVelocity(tau, -(tphDp - tAtVc), true);
			s = deaccelPlusPath.getPosition(tau, (tphDp - tAtVc), ((tphDp - tAtVc) * (tphDp - tAtVc)) );
		}

		trS->set(s);
		trA->set(a);
		trV->set(v);

		double intA = integral.process(a);
		trVI->set(intA);
		trSI->set(integral2.process(v));
		//trSI->set(integral2.process(intA));

		trErrV->set(trVI->get() - v);
		trErrS->set(trSI->get() - s);
		tracer.trace();
		tracer2.trace();
	}
	
	double velocityAtDp = (Vc - abs(velocityDp));
	integral.reset();

	integral.setInitialConditions(velocityAtDp);

	integral2.reset();
	integral2.setInitialConditions(posAtStartDp + abs(distToDo) + distanceDp);

	//phase Dc (constant deacceleration)

	/*const acceleration*/
	
	std::shared_ptr<ConstFunction> mathFuncConstD = std::make_shared<ConstFunction>(deltaD);
	PathSegment constDeaccelPath(tphDp, tphDc, 0.00, velocityAtDp, posAtStartDp + abs(distToDo) + distanceDp, mathFuncConstD);

	for (double t = (tphDp + 0.0001); t < (tphDc + 0.000001); t = t + 0.0001)
	{		
		a = constDeaccelPath.getAccel(t);
		v = constDeaccelPath.getVelocity(t);
		s = constDeaccelPath.getPosition(t);
			
		trS->set(s);
		trA->set(a);
		trV->set(v);

		double intA = integral.process(a);
		trVI->set(intA);
		trSI->set(integral2.process(v));
		//trSI->set(integral2.process(intA));


		trErrV->set(trVI->get() - v);
		trErrS->set(trSI->get() - s);

		tracer.trace();
		tracer2.trace();
	}
	integral.reset();
	integral.setInitialConditions(velocityDp);
		
	integral2.reset();
	integral2.setInitialConditions(posAtStartDp + abs(distToDo) + distanceDp + distanceDc);

	
	//phase D-

	double endStartPos = posAtStartDp + abs(distToDo) + distanceDp + distanceDc;

	PathSegment deaccelMinusPath(0.00, 1.00, 0.00, velocityDp, endStartPos, funcDminus);

	double isFinal = deaccelMinusPath.getPosition(1.00, tphDm - tphDc, (tphDm - tphDc) * (tphDm - tphDc), true);
	double test11 = f_pos - isFinal;
	double lastScaling = 0.00;
	if (test11 <= 0.00)
	{
		lastScaling = f_pos / isFinal;
	}
	else
	{
		lastScaling = isFinal / f_pos;
	}

	for (double t = (tphDc + 0.0001); t <= (tphDm + 0.000001); t = t + 0.0001)
	{
		double delta_t = (t - tphDc);
		double tau = delta_t * time_scale_factor2;
		double tp = tphDm - tphDc;
		
		if (tau <= 1.00)
		{
			a = deaccelMinusPath.getAccel(tau);
			v = deaccelMinusPath.getVelocity(tau, tp, true);			
			s = deaccelMinusPath.getPosition(tau, tp, tp * tp, true) * lastScaling;
		}

		trS->set(s);
		trA->set(a);
		trV->set(v);

		double intA = integral.process(a);
		trVI->set(intA);
		trSI->set(integral2.process(v));
		//trSI->set(integral2.process(intA));

		trErrV->set(trVI->get() - v);
		trErrS->set(trSI->get() - s);
		tracer.trace();
		tracer2.trace();
	}

}

TEST(TestHepticPolynomial, TestHepticPolynomial1)
{
	using namespace Math;
	using namespace Polynomials;
	using namespace TrajectoryGeneration;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/HepticPolynomial.dat";

	double ts = 0.001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto xt = tracer.addSignal<std::double_t>("x", BaseSignal::SignalType::Double);
	auto x1t = tracer.addSignal<std::double_t>("x_der_1", BaseSignal::SignalType::Double);
	auto x2t = tracer.addSignal<std::double_t>("x_der_2", BaseSignal::SignalType::Double);
	auto x3t = tracer.addSignal<std::double_t>("x_der_3", BaseSignal::SignalType::Double);

	double x = 0.00;
	double x1 = 0.00;
	double x2 = 0.00;
	double x3 = 0.00;
	double x4 = 0.00;

	double tf = 0.1;
	
	double i_pos = 0.00;
	double i_vel = 0.00;
	double i_accel = 0.00;
	double f_pos = 300.00;
	double f_vel = 0.00;
	double f_accel = 0.00;

	double v_max = 300.00;
	double a_max = 600.00;
	double j_max = 6000.00;

		
	SimplelHepticTrajectory sht(i_pos, f_pos, v_max, a_max, tf);
	double time = sht.calculateMinTime(0.5, 0.5);
	tf = time;
	EXPECT_TRUE(time > 0.00001 );
	sht.create(time);


	for (double t = 0.00001; t <= tf; t = t + 0.001)
	{
		sht.calculateValuesForTime(t, x, x1, x2, x3);
		xt->set(x);
		x1t->set(x1);
		x2t->set(x2);
		
		tracer.trace();
	}
}

TEST(TestHexicPolynomial, TestHexicPolynomial1)
{
	using namespace Math;
	using namespace Polynomials;
	HexicPolynomial poly;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/HexicPolynomial.dat";

	double ts = 0.001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());


	auto xt = tracer.addSignal<std::double_t>("x", BaseSignal::SignalType::Double);
	auto x1t = tracer.addSignal<std::double_t>("x_der_1", BaseSignal::SignalType::Double);
	auto x2t = tracer.addSignal<std::double_t>("x_der_2", BaseSignal::SignalType::Double);
	auto x3t = tracer.addSignal<std::double_t>("x_der_3", BaseSignal::SignalType::Double);

	double x = 0.00;
	double x1 = 0.00;
	double x2 = 0.00;
	double x3 = 0.00;
	double x4 = 0.00;

	double tf = 1.50;


	double a0 = 0.00;
	double a1 = 0.00;
	double a2 = 0.00;
	double a3 = 0.00;
	double a4 = 0.00;
	double a5 = 0.00;
	double a6 = 0.00;

	double i_pos = 0.00;
	double i_vel = 100.00;
	double i_accel = 10.00;
	double f_pos = 300.00;
	double f_vel = 200.00;
	double f_accel = 0.00;

	double v_max = 300.00;
	double a_max = 600.00;

	a0 = i_pos;
	a1 = i_vel;;
	a2 = i_accel/2.00;

	double tau, tau_2, tau_3, tau_4, tau_5, tau_6;

	tau = 1.00;
	tau_2 = tau * tau;
	tau_3 = tau_2 * tau;
	tau_4 = tau_3 * tau;
	tau_5 = tau_4 * tau;
	tau_6 = tau_5 * tau;

	double tf_2 = tf * tf;
	double tf_3 = tf_2 * tf;
	double tf_4 = tf_3 * tf;
	double tf_5 = tf_4 * tf;
	double tf_6 = tf_5 * tf;

	Eigen::Matrix3d A;
	A << tau_3, tau_4, tau_5,
		4.00 * tau_3, 4.00 * tau_3, 5.00 * tau_4/*, 6.00 * tau_5*/,
		6.00 * tau, 12.00 * tau_2, 20.00 * tau_3;/* 30.00 * tau_4*/
		//6.00, 24.00 * tau, 60.00 * tau_2;/*, 120.00 * tau_3*/
		

	Eigen::Vector3d b(f_pos - i_pos - i_vel * tau - 0.50 * i_accel * tau * tau,
		f_vel - i_vel - i_accel * tau,
		f_accel - i_accel);


	//(f_pos - i_pos + (i_vel - f_vel ) * tf - (i_accel - f_accel) * tau_5);



	Eigen::Vector3d vec_x = A.colPivHouseholderQr().solve(b);


	//*delta_s / (tf_4);

	a3 = vec_x(0) / tf_3;;
	a4 = vec_x(1)/ tf_4;
	a5 = vec_x(2)/ tf_5;
	a6 = 0.00;

	/*
	Eigen::Matrix3d A;

		A << tf_4, tf_5, tf_6,
		4.00 * tf_3, 5.00 * tf_4, 6.00 * tf_5,
		12.00 * tf_2, 20.00 * tf_3, 30.00 * tf_4;
		
	

	Eigen::Vector3d b(f_pos - i_pos - i_vel * tf - 0.50 * i_accel * tf * tf,
		f_vel - i_vel - i_accel * tf,
		f_accel - i_accel);
		// Compute the least squares solution
	Eigen::Vector3d vec_x = A.colPivHouseholderQr().solve(b);

	
	a3 = 0.00;
	a4 = vec_x(0);
	a5 = vec_x(1);
	a6 = vec_x(2);
	*/

	

	poly.setParams(a0, a1, a2, a3, a4, a5, a6);

	for (double t = 0.00001; t <= tf+0.1; t = t + 0.001)
	{
		poly.calculate(t, x, x1, x2, x3, x4);
		xt->set(x);
		x1t->set(x1);
		x2t->set(x2);
		x3t->set(x3);
		tracer.trace();
	}
}


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

TEST(TestImportExport, TestTankImportExport)
{
	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/tank.fis";
	std::string fileName2 = strpath + "test/tank_exported.fis";

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

	std::ofstream file(fileName2);

	if (!file)
	{
		ossErrors << "Error creating file " << std::endl;
	}

	std::ifstream file1(fileName1);

	FisFileExport fexp(controllerFromFis, file);

	EXPECT_TRUE(fexp.exportToFIS());
	//to do compare two files

	file.close();

	std::ifstream file2(fileName2);

	file2.seekg(0, std::ifstream::beg);
	file1.seekg(0, std::ifstream::beg);
	bool equ =  std::equal(std::istreambuf_iterator<char>(file2.rdbuf()),
		std::istreambuf_iterator<char>(),
		std::istreambuf_iterator<char>(file1.rdbuf()));

	EXPECT_TRUE(equ);
}


TEST(TestImportExport, TestSugenoImportExport1)
{
	std::ostringstream ossErrors;
	
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/sugeno1.fis";
	std::string fileName2 = strpath + "test/sugeno1_exported.fis";

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

	//test controller

	FuzzyInput* voltage = controllerFromFis->getInput("voltage");
	FuzzyInput* temperature = controllerFromFis->getInput("temperature");
	FuzzyOutput* output1 = controllerFromFis->getOutput();

	std::double_t out1;

	controllerFromFis->compile();
		
	//compare wit expected values
	
	voltage->setValue(0.5);
	temperature->setValue(0.0);
	controllerFromFis->process();
	out1 = output1->getValue();
	EXPECT_FLOAT_EQ(out1, 0.20);

	voltage->setValue(0.33);
	temperature->setValue(-0.228);
	controllerFromFis->process();
	out1 = output1->getValue();
	EXPECT_FLOAT_EQ(out1, 0.255031349);

	voltage->setValue(0.756);
	temperature->setValue(-0.45);
	controllerFromFis->process();
	out1 = output1->getValue();
	EXPECT_FLOAT_EQ(out1, 0.575476455646493);

	voltage->setValue(0.87);
	temperature->setValue(-0.609);
	controllerFromFis->process();
	out1 = output1->getValue();
	EXPECT_FLOAT_EQ(out1, 0.90543030856040529);

	voltage->setValue(0.365);
	temperature->setValue(-0.609);
	controllerFromFis->process();
	out1 = output1->getValue();
	EXPECT_FLOAT_EQ(out1, 0.56552098940765594);


	std::ofstream file(fileName2);

	if (!file)
	{
		ossErrors << "Error creating file " << std::endl;
	}

	std::ifstream file1(fileName1);

	FisFileExport fexp(controllerFromFis, file);

	EXPECT_TRUE(fexp.exportToFIS());
	//to do compare two files



	file.close();

	std::ifstream file2(fileName2);

	file2.seekg(0, std::ifstream::beg);
	file1.seekg(0, std::ifstream::beg);
	bool equ = std::equal(std::istreambuf_iterator<char>(file2.rdbuf()),
		std::istreambuf_iterator<char>(),
		std::istreambuf_iterator<char>(file1.rdbuf()));

	EXPECT_TRUE(equ);



}




TEST(TestImportExport, TestInput1)
{
	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/test_input1.fis";
	std::string fileName2 = strpath + "test/test_input1_exported.fis";

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

	std::ofstream file(fileName2);

	if (!file)
	{
		ossErrors << "Error creating file " << std::endl;
	}

	std::ifstream file1(fileName1);

	FisFileExport fexp(controllerFromFis, file);

	EXPECT_TRUE(fexp.exportToFIS());
	//to do compare two files

	file.close();

	std::ifstream file2(fileName2);

	file2.seekg(0, std::ifstream::beg);
	file1.seekg(0, std::ifstream::beg);
	bool equ = std::equal(std::istreambuf_iterator<char>(file2.rdbuf()),
		std::istreambuf_iterator<char>(),
		std::istreambuf_iterator<char>(file1.rdbuf()));

	EXPECT_TRUE(equ);
}

TEST(TestCreateAndExport, TestCreate)
{
	/*define input variables */

	std::unique_ptr<FuzzyInput> temperature = std::make_unique<FuzzyInput>(-80.00, 80.00, "Temperature");

	std::unique_ptr<FuzzySet> veryLow = std::make_unique<TrapezoidalIFuzzySetInfL>(-15.0, -3.0, "VeryLow");
	std::unique_ptr<FuzzySet> low = std::make_unique<TriangularFuzzySet>(-5.0, 0.0, 18.0, "Low");
	std::unique_ptr<FuzzySet> normal = std::make_unique<TriangularFuzzySet>(18, 24, 30, "Normal");
	std::unique_ptr<FuzzySet> high = std::make_unique<TriangularFuzzySet>(28.0, 40.0, 52.0, "High");
	std::unique_ptr<FuzzySet> veryHigh = std::make_unique<TrapezoidalIFuzzySetInfR>(50.0, 60.0, "VeryHigh");

	temperature->addFuzzySet(std::move(veryLow));
	temperature->addFuzzySet(std::move(low));
	temperature->addFuzzySet(std::move(normal));
	temperature->addFuzzySet(std::move(high));
	temperature->addFuzzySet(std::move(veryHigh));

	std::unique_ptr<FuzzyInput> dcBusVoltage = std::make_unique<FuzzyInput>(-10.0, 900.00, "Voltage");

	std::unique_ptr<FuzzySet> veryLowVoltage = std::make_unique<TrapezoidalIFuzzySetInfL>(0.0, 40, "Low");
	std::unique_ptr<FuzzySet> normalVoltage = std::make_unique<TriangularFuzzySet>(30, 120, 200, "Normal");
	std::unique_ptr<FuzzySet> highVoltage = std::make_unique<TrapezoidalIFuzzySetInfR>(195, 240, "High");

	dcBusVoltage->addFuzzySet(std::move(veryLowVoltage));
	dcBusVoltage->addFuzzySet(std::move(normalVoltage));
	dcBusVoltage->addFuzzySet(std::move(highVoltage));

	std::unique_ptr<FuzzyOutput> pwmSignal = std::make_unique<FuzzyOutput>(0.0, 1.00, "PWM");

	std::unique_ptr<FuzzySet> veryLowOutput = std::make_unique<TrapezoidalIFuzzySetInfL>(0.0, 0.3, "Low");
	std::unique_ptr<FuzzySet> normalOutput = std::make_unique<TriangularFuzzySet>(0.25, 0.5, 0.75, "Normal");
	std::unique_ptr<FuzzySet> highOutput = std::make_unique<TrapezoidalIFuzzySetInfR>(0.7, 1.0, "High");

	pwmSignal->addFuzzySet(std::move(veryLowOutput));
	pwmSignal->addFuzzySet(std::move(normalOutput));
	pwmSignal->addFuzzySet(std::move(highOutput));

	FuzzyController* controller = FuzzyControllerFactory::createController(FuzzyControllerType::Mamdani, "Fuzzy Logic PID Controller");

	controller->addInput(std::move(temperature));
	controller->addInput(std::move(dcBusVoltage));
	controller->addOutput(std::move(pwmSignal));

	std::vector<std::string> rv = { "Temperature", "Voltage" };

	// if (Temperature is more or less VeryLow) or (Voltage is Low) then PWM is Low
	std::vector<std::vector<std::string>> vecHedgesR1;
	std::vector<std::string> vecHedgesR1T;
	std::vector<std::string> vecHedgesR1V;
	vecHedgesR1T.push_back("is");
	vecHedgesR1T.push_back("more or less");

	vecHedgesR1V.push_back("is");

	vecHedgesR1.push_back(vecHedgesR1T);
	vecHedgesR1.push_back(vecHedgesR1V);

	std::vector<std::string> rl1 = { "VeryLow",		"Low" };
	std::vector<std::string> rl2 = { "Low",			"Low" };
	std::vector<std::string> rl3 = { "Normal",			"Low" };
	std::vector<std::string> rl4 = { "VeryLow",			"Normal" };
	std::vector<std::string> rl5 = { "Low",				"Normal" };
	std::vector<std::string> rl6 = { "Normal",			"Normal" };
	std::vector<std::string> rl7 = { "High",			"Normal" };

	controller->addRule(BooleanOperation::AndMin, "R1", "", rv, vecHedgesR1, rl1, "PWM", "Low", 1.00);

	// if (Temperature is VeryLow) and (Voltage is Normal) then PWM is High
	std::vector<std::vector<std::string>> vecHedgesR2;
	std::vector<std::string> vecHedgesR2T;
	std::vector<std::string> vecHedgesR2V;
	vecHedgesR2T.push_back("is");
	vecHedgesR2V.push_back("is");
	vecHedgesR2.push_back(vecHedgesR2T);
	vecHedgesR2.push_back(vecHedgesR2V);

	controller->addRule(BooleanOperation::AndMin, "R2", "", rv, vecHedgesR2, rl2, "PWM", "Low", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R4", "", rv, vecHedgesR2, rl4, "PWM", "High", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R3", "", rv, vecHedgesR2, rl3, "PWM", "Low", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R5", "", rv, vecHedgesR2, rl5, "PWM", "High", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R7", "", rv, vecHedgesR2, rl7, "PWM", "Normal", 1.00);

	controller->addRule(BooleanOperation::AndMin, "R6", "", rv, vecHedgesR2, rl6, "PWM", "High", 1.00);


	FuzzyInput* inTemp = controller->getInput("Temperature");
	FuzzyInput* inVolt = controller->getInput("Voltage");
	FuzzyOutput* outPWM = controller->getOutput();

	controller->compile();

	std::double_t t1 = -20;
	std::double_t v1 = 20.00;

	std::double_t val;

	struct RES
	{
		std::double_t temperatur;
		std::double_t voltage;
		std::double_t value;
	};
	std::vector<RES> results;
	RES res;
	for (int i = 0; i < 100; i++)
	{
		t1 = t1 + 1.0;
		v1 = v1 + 0.7;
		res.temperatur = t1;
		res.voltage = v1;
		inTemp->setValue(t1);
		inVolt->setValue(v1);
		controller->process();
		val = outPWM->getValue();
		res.value = val;
		results.push_back(res);
	}
}
/*
TEST(CompareWithReference, TestTank2)
{
	
	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	std::string fileName1 = strpath + "test/tank2.fis";
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

			std::double_t diff = abs(o1 - r1);
			if (diff > 0.0002)
			{
				EXPECT_FLOAT_EQ(diff, 0);
			}
		}
	}
}
*/

/*

TEST(TestCaseFrictionDCMotor, DCMotorFuzzy)
{
	std::ostringstream ossErrors;
	auto p = std::filesystem::current_path();

	std::string strpath = p.generic_string();

	StringUtil::remove_substring(strpath, "CntrlLibraryTest");

	strpath = strpath + "test/vfriction.fis";

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


	//test controller

	FuzzyInput* inError = controllerFromFis->getInput("error");
	FuzzyInput* rate = controllerFromFis->getInput("rate");
	FuzzyOutput* output1 = controllerFromFis->getOutput();

	std::double_t out1;

	controllerFromFis->compile();
	
	using namespace Models;

	DCMotor motor;
	std::double_t J = 0.008586328125;

	std::double_t Ki1 = 1904.96918720126;
	std::double_t Ki2 = 30.2;

	motor.setParameters(0.0001, 0.000135, 0.178, J, 1.1, 1.55e-3, 0.7614);

	PIDController piTq;
	piTq.setParameters(2.00174495936295, Ki1, 0.00, Ki1, 0.0001, 120.0);


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
	std::double_t refVel = 0.9;
	std::double_t u = 0.00;


	std::vector< std::double_t> vecVel;
	std::vector< std::double_t> vecU;
	std::vector< std::double_t> vecTr;
	std::vector< std::double_t> vecTime;
	std::double_t time = 0.00;
	std::double_t rateVal = 0.00;
	Derivative der;
	der.setParameters(0.0001, 1.0);



	inError->setValue(0.0);
	rateVal = der.process(0.0);
	rate->setValue(rateVal);
	controllerFromFis->process();
	out1 = output1->getValue();

	for (std::uint32_t k = 0; k < 10000; k++)
	{
		//velocity controller
		errorVel = (refVel - w);

		if (errorVel < 2.0)
		{
			int a = 0;
			a++;
		}

		errorVel = errorVel / 400.00;
			
		inError->setValue(errorVel);
		rateVal = der.process(errorVel) / 5000.00;
		rate->setValue(rateVal);
		controllerFromFis->process();
		out1 = output1->getValue();

		Tref = -out1 * 600.00;

		if (Tref > 40)
		{
			Tref = 40.00;
		}

		if (Tref < -40.00)
		{
			Tref = -40.00;
		}

		vecTr.push_back(Tref);

		//torque controller
		error = Tref - T;
		u = piTq.process(error);

		motor.setInputs(u, Tf);
		motor.process();
		I = motor.getCurrent();
		w = motor.getVelocity();
		a = motor.getAccell();
		T = motor.getTorque();

		vecVel.push_back(w);
		vecU.push_back(u);

		vecTime.push_back(time);
		time += 0.0001;

		friction.setInputs(w, T, a);
		friction.process();
		Tf = friction.getFrictionTorque();
	}
}
*/
