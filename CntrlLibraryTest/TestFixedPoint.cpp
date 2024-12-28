#include "pch.h"


#include "FixedPoint.h"
#include "ParkClarke.h"
#include "CircleLimitation.h"
#include "QPIController.h"
#include "PIDController.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace CntrlLibrary;


int16_t PI_Controller(int32_t wProcessVarError)
{
	int16_t returnValue;

	static int32_t wIntegralTermOld = 0.00;
	
	int32_t wIntegral_Term;
	int32_t wOutput_32;
	int32_t wIntegral_sum_temp;
	int32_t wDischarge = 0;
	int16_t hUpperOutputLimit = (int32_t)(INT16_MAX * 4096);
	int16_t hLowerOutputLimit = (int32_t)(-INT16_MAX * 4096);

	int16_t    hDefKpGain = (int16_t)2312;
	int16_t    hDefKiGain = (int16_t)2158;

	uint16_t hKpDivisorPOW2 = (uint16_t)7;
	uint16_t hKiDivisorPOW2 = (uint16_t)12;

	int32_t wProportional_Term = hDefKpGain * wProcessVarError;
	wIntegral_Term = hDefKiGain * wProcessVarError + wIntegralTermOld;
	wOutput_32 = (wProportional_Term >> hKpDivisorPOW2) + (wIntegralTermOld >> hKiDivisorPOW2);
	wIntegralTermOld = wIntegral_Term;
	returnValue = (int16_t)wOutput_32;
	return (returnValue);
}

TEST(TestQNumbers, TestMathOperations)
{
	Q15 a(0.5f);

	Q15 b(1.0f);

	Q15 r1 = a + a;
	float rf1 = r1.toFloat();

	Q15 r2 = Q15(1.0f) * a;
	float rf2 = r1.toFloat();

	AB_t<Q15> input;
	input.a = Q15(-0.05f);
	input.b = Q15(0.05f);

	AlphaBeta_t res = ParkClarke<Q15>::Clarke(input);

	float rAlpha = res.alpha.toFloat();
	float rBeta = res.beta.toFloat();

	EXPECT_NEAR(-0.05f, res.alpha.toFloat() , 0.02);
	EXPECT_NEAR(0.0288675f, rBeta, 0.02);

	AB_t out = ParkClarke<Q15>::InverseClarke(res);
	EXPECT_NEAR(input.a.raw(), out.a.raw(), 1);
	EXPECT_NEAR(input.b.raw(), out.b.raw(), 1);
	
	input.a = Q15(0.10f);
	input.b = Q15(0.05f);
	res = ParkClarke<Q15>::Clarke(input);
	out = ParkClarke<Q15>::InverseClarke(res);
	EXPECT_NEAR(input.a.raw(), out.a.raw(), 1);
	EXPECT_NEAR(input.b.raw(), out.b.raw(), 1);
	
	input.a = Q15(0.10f);
	input.b = Q15(-0.05f);
	res = ParkClarke<Q15>::Clarke(input);
	out = ParkClarke<Q15>::InverseClarke(res);
	EXPECT_NEAR(input.a.raw(), out.a.raw(), 1);
	EXPECT_NEAR(input.b.raw(), out.b.raw(), 1);


	input.a = Q15(-0.05f);
	input.b = Q15(0.05f);
	res = ParkClarke<Q15>::Clarke(input);
	out = ParkClarke<Q15>::InverseClarke(res);
	rBeta = res.beta.toFloat();

	EXPECT_NEAR(input.a.raw(), out.a.raw(), 1);
	EXPECT_NEAR(input.b.raw(), out.b.raw(), 1);


	QD_t<Q15> qd;
	AlphaBeta_t<Q15> alphaBeta;
	TrigComponents trig;
	
	alphaBeta.alpha = Q15(1.0f);
	alphaBeta.beta = Q15(0.0f);
	trig.hSin = Q15(sin(0.0f));
	trig.hCos = Q15(cos(0.0f));
	
	qd = ParkClarke<Q15>::Park(alphaBeta, trig);
	EXPECT_NEAR(1.0f, qd.d.toFloat(), 0.01);
	EXPECT_NEAR(0.0f, qd.q.toFloat(), 0.01);
	res = ParkClarke<Q15>::InvPark(qd, trig);
	EXPECT_NEAR(res.alpha.raw(), alphaBeta.alpha.raw(), 5);
	EXPECT_NEAR(res.beta.raw(), alphaBeta.beta.raw(), 5);


	alphaBeta.alpha = Q15(1.0f);
	alphaBeta.beta = Q15(0.0f);
	trig.hSin = Q15(sin(3.14f / 2.0f));
	trig.hCos = Q15(cos(3.14f / 2.0f));

	qd = ParkClarke<Q15>::Park(alphaBeta, trig);
	EXPECT_NEAR(0.0f, qd.d.toFloat(), 0.01);
	EXPECT_NEAR(-1.0f, qd.q.toFloat(), 0.01);
	res = ParkClarke<Q15>::InvPark(qd, trig);
	EXPECT_NEAR(res.alpha.raw(), alphaBeta.alpha.raw(), 2);
	EXPECT_NEAR(res.beta.raw(), alphaBeta.beta.raw(), 2);


	alphaBeta.alpha = Q15(1.0f);
	alphaBeta.beta = Q15(1.0f);
	trig.hSin = Q15(sin(3.14f / 4.0f));
	trig.hCos = Q15(cos(3.14f / 4.0f));

	qd = ParkClarke<Q15>::Park(alphaBeta, trig);
	EXPECT_NEAR(1.414f, qd.d.toFloat(), 0.01);
	EXPECT_NEAR(0.0f, qd.q.toFloat(), 0.01);
	res = ParkClarke<Q15>::InvPark(qd, trig);
	EXPECT_NEAR(res.alpha.raw(), alphaBeta.alpha.raw(), 3);
	EXPECT_NEAR(res.beta.raw(), alphaBeta.beta.raw(), 3);
	

	alphaBeta.alpha = Q15(0.0f);
	alphaBeta.beta = Q15(1.0f);
	trig.hSin = Q15(sin(3.14f / 2.0f));
	trig.hCos = Q15(cos(3.14f / 2.0f));

	qd = ParkClarke<Q15>::Park(alphaBeta, trig);
	EXPECT_NEAR(1.0f, qd.d.toFloat(), 0.01);
	EXPECT_NEAR(0.0f, qd.q.toFloat(), 0.01);
	res = ParkClarke<Q15>::InvPark(qd, trig);
	EXPECT_NEAR(res.alpha.raw(), alphaBeta.alpha.raw(), 2);
	EXPECT_NEAR(res.beta.raw(), alphaBeta.beta.raw(), 2);


	Q15 aa(0.5f);
	Q31 bb(0.25f);


	float aaf = aa.toFloat();
	float bbf = bb.toFloat();


	auto result1 = FixedPointOps::add(aa, bb);
	float f1 = result1.toFloat();

	auto result2 = FixedPointOps::sub(aa, bb);
	float f2 = result2.toFloat();


	alphaBeta.alpha = Q15(0.5f);
	alphaBeta.beta = Q15(0.5f);
	trig.hSin = Q15(sin(3.14f / 3.0f));
	trig.hCos = Q15(cos(3.14f / 3.0f));

	qd = ParkClarke<Q15>::Park(alphaBeta, trig);
	EXPECT_NEAR(0.683f, qd.d.toFloat(), 0.01);
	EXPECT_NEAR(-0.183f, qd.q.toFloat(), 0.01);
	res = ParkClarke<Q15>::InvPark(qd, trig);
	EXPECT_NEAR(res.alpha.raw(), alphaBeta.alpha.raw(), 2);
	EXPECT_NEAR(res.beta.raw(), alphaBeta.beta.raw(), 2);


	alphaBeta.alpha = Q15(1.0f);
	alphaBeta.beta = Q15(-1.0f);

	Q15 testJump = alphaBeta.alpha - alphaBeta.beta;
	float ftestJump = testJump.toFloat();
	Q15 testJump2(2.0f);
	float ftestJump2 = testJump2.toFloat();

	Q15 previousAngle(0.99f);
	Q15 rotorAngle(-0.99f);
	auto deltaAngle = rotorAngle - previousAngle;

	// Normalize the angle difference to handle wraparound
	if (deltaAngle > Q15(1.0f))
	{
		deltaAngle -= Q15(2.0f); // Correct for jump from -1 to 1
	}
	else if (deltaAngle < Q15(-1.0f))
	{
		deltaAngle += Q15(2.0f); // Correct for jump from 1 to -1
	}
	float fdeltaAngle = deltaAngle.toFloat();




	trig.hSin = Q15(sin(-3.14f / 4.0f));
	trig.hCos = Q15(cos(-3.14f / 4.0f));

	qd = ParkClarke<Q15>::Park(alphaBeta, trig);
	EXPECT_NEAR(1.414f, qd.d.toFloat(), 0.01);
	EXPECT_NEAR(0.0f, qd.q.toFloat(), 0.01);
	res = ParkClarke<Q15>::InvPark(qd, trig);
	EXPECT_NEAR(res.alpha.raw(), alphaBeta.alpha.raw(), 2);
	EXPECT_NEAR(res.beta.raw(), alphaBeta.beta.raw(), 2);

	CircleLimitation<Q15> limiter(Q15(1.0f), Q15(0.95f));
	QD_t<Q15> testLimitValue;
	testLimitValue.q = Q15(0.9f);
	testLimitValue.d = Q15(0.9f);
	float unMax = sqrt(testLimitValue.q.toFloat() * testLimitValue.q.toFloat() + testLimitValue.d.toFloat() * testLimitValue.d.toFloat());
	QD_t<Q15>  limited = limiter.calculateSaturation(testLimitValue);
	float fLimQ = limited.q.toFloat();
	float fLimD = limited.d.toFloat();
	float unLimited = sqrt(fLimQ * fLimQ + fLimD * fLimD);
	EXPECT_NEAR(unLimited, 1.00, 0.02);

	testLimitValue.q = Q15(-0.9f);
	testLimitValue.d = Q15(-0.9f);
	unMax = sqrt(testLimitValue.q.toFloat() * testLimitValue.q.toFloat() + testLimitValue.d.toFloat() * testLimitValue.d.toFloat());
	limited = limiter.calculateSaturation(testLimitValue);
	fLimQ = limited.q.toFloat();
	fLimD = limited.d.toFloat();
	unLimited = sqrt(fLimQ * fLimQ + fLimD * fLimD);
	EXPECT_NEAR(unLimited, 1.00, 0.02);

	testLimitValue.q = Q15(0.9f);
	testLimitValue.d = Q15(1.0f);
	unMax = sqrt(testLimitValue.q.toFloat() * testLimitValue.q.toFloat() + testLimitValue.d.toFloat() * testLimitValue.d.toFloat());
	limited = limiter.calculateSaturation(testLimitValue);
	fLimQ = limited.q.toFloat();
	fLimD = limited.d.toFloat();
	unLimited = sqrt(fLimQ * fLimQ + fLimD * fLimD);
	EXPECT_NEAR(unLimited, 1.00, 0.02);


	testLimitValue.q = Q15(-0.9f);
	testLimitValue.d = Q15(-1.0f);
	unMax = sqrt(testLimitValue.q.toFloat() * testLimitValue.q.toFloat() + testLimitValue.d.toFloat() * testLimitValue.d.toFloat());
	limited = limiter.calculateSaturation(testLimitValue);
	fLimQ = limited.q.toFloat();
	fLimD = limited.d.toFloat();
	unLimited = sqrt(fLimQ * fLimQ + fLimD * fLimD);
	EXPECT_NEAR(unLimited, 1.00, 0.02);

	Q9_7 tes97(200.15f);
	Q9_7 current(210.5f);
	auto rt1 = FixedPointOps::mul( tes97,  current);
	float ttt = rt1.toFloat();

	Q9_7 sc(15.23f);
	Q10_9 dst;
	FixedPointOps::convert(sc, dst);
	float dstR = dst.toFloat();

	float kp = 18.0625f;
	Q9_7 tttt(kp);
	float sfdddf = tttt.toFloat();

	float ki = 0.525685f;
	float kb = 1.1f;
	
	float upsat = 120.0f;
	QPIController< Q9_7,Q9_7, Q4_12, Q9_7> piController( kp, ki, kb, upsat);
	DiscreteTime::PIDController pidFp(kp, ki, 0.0f, kb, 1.0f, upsat);

	float ferr = 0.5f;
	Q9_7 error( ferr);
	Q9_7 piout = piController.process(error);
	float fResPi = piout.toFloat();
	float fresPidF = pidFp.process(ferr);

 	int16_t i16Res = PI_Controller(65536/128);

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(65536 / 128);

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(65536 / 128);

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(65536 / 128);

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(65536 / 128);

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(65536 / 128);

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(error.raw());

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(error.raw());

	piout = piController.process(error);
	fResPi = piout.toFloat();
	fresPidF = pidFp.process(ferr);
	i16Res = PI_Controller(error.raw());	

}