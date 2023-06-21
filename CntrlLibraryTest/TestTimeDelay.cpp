#include "pch.h"

#include "TimeDelay.h"

using namespace CntrlLibrary;

using namespace CntrlLibrary::DiscreteTime;

TEST(TestCaseTimeDelay, TimeDelay)
{
	TimeDelay timeDelay;

	timeDelay.setParameters(5U);

	EXPECT_EQ(0.00, timeDelay.process(1.0));
	EXPECT_EQ(0.00, timeDelay.process(2.0));
	EXPECT_EQ(0.00, timeDelay.process(3.0));
	EXPECT_EQ(0.00, timeDelay.process(4.0));
	EXPECT_EQ(0.00, timeDelay.process(5.0));
	EXPECT_EQ(1.00, timeDelay.process(6.0));
	EXPECT_EQ(2.00, timeDelay.process(7.0));
	EXPECT_EQ(3.00, timeDelay.process(8.0));
	EXPECT_EQ(4.00, timeDelay.process(9.0));
	EXPECT_EQ(5.00, timeDelay.process(10.0));
	EXPECT_EQ(6.00, timeDelay.process(11.0));
	
	timeDelay.reset();
	EXPECT_EQ(0.00, timeDelay.process(1.0));
}