#include "pch.h"


#include "FixedPoint.h"
#include "ParkClarke.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace CntrlLibrary;


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
	trig.hSin = Q15(sin(-3.14f / 4.0f));
	trig.hCos = Q15(cos(-3.14f / 4.0f));

	qd = ParkClarke<Q15>::Park(alphaBeta, trig);
	EXPECT_NEAR(1.414f, qd.d.toFloat(), 0.01);
	EXPECT_NEAR(0.0f, qd.q.toFloat(), 0.01);
	res = ParkClarke<Q15>::InvPark(qd, trig);
	EXPECT_NEAR(res.alpha.raw(), alphaBeta.alpha.raw(), 2);
	EXPECT_NEAR(res.beta.raw(), alphaBeta.beta.raw(), 2);



}