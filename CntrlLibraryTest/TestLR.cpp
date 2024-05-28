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

#include "PIDController.h"
#include "StringUtil.h"
#include "IIRFilterFO.h"
#include "LRCircuit.h"
#include "WaveFormTracer.h"


using namespace CntrlLibrary;

using std::experimental::filesystem::path;

using namespace DiscreteTime;
TEST(TestCaseLR, TestLRController)
{
	
	Models::LRCircuit lrCircuit;
	
	
	std::double_t ts = 0.01; //10 ms
	//System parameters

	std::double_t R = 20.00; //Coil resistance in [Ohm]
	std::double_t L = 0.1; //Inductance in [H]
	std::double_t U = 24.00;
	lrCircuit.setParameters(0.0001, L, R); //Ts of LR is 100uS 


	
	PIDController piCntrl;
	Filters::IIRFirstOrderFilter preFilter;

	std::double_t kp = 1.44;
	std::double_t ki = 34.11;
	std::double_t kd = 0.00;
	std::double_t kb = 1.00;

	//First order IIR Filter coefficients
	std::double_t wc = 60.5; // rad/sec
	std::double_t alpha = exp(-wc * ts); //rad/sec
	std::double_t iirPre_a1 = -alpha;
	std::double_t iirPre_b0 = 1 - alpha;
	std::double_t iirPre_b1 = 0.00; // first order
	preFilter.setParameters(iirPre_a1, iirPre_b0, iirPre_b1);

	std::double_t upSaturation = 1.0;
	std::double_t lowSaturation = 0.0;

	//max bandwith is dependant from motor electrical constant
	//std::double_t tau_e = L / R;
		
	piCntrl.setParameters(kp, ki, kd, kb, ts, upSaturation, lowSaturation);
	
	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestLR.dat";

	WaveFormTracer tracer(fileName1, 0.0001);
	EXPECT_TRUE(tracer.open());

	auto lrIshPtr = tracer.addSignal<std::double_t>("I", BaseSignal::SignalType::Double);
	auto refIShPtr = tracer.addSignal<std::double_t>("refI", BaseSignal::SignalType::Double);
	auto measIShPtr = tracer.addSignal<std::double_t>("Iadc", BaseSignal::SignalType::Double);
	

	refIShPtr->set(1.00);  // 1000 mA

	tracer.writeHeader();
	double i_meas(0.00);

	double res = 2.00 / 4096;

	for (std::uint32_t i = 0U; i < 25; i++)
	{		
		double error = preFilter.process(refIShPtr->get()) - i_meas;
		//double error = refIShPtr->get() - i_meas;
		double duty_cycle = piCntrl.process(error);
	
		
		i_meas = 0.00;
		for (int n = 0; n < 100; n++)
		{
			//calculat e k based of duty_cycle
			int k = (int)(duty_cycle * 100.0);
			if (n < k)
			{
				lrCircuit.setInput(U);
			}
			else
			{
				lrCircuit.setInput(0.00);
			}
			lrCircuit.process();
			double i_c = lrCircuit.getI();

			int nadc = (int)(i_c / res);
			i_c = nadc * res;

			 
			lrIshPtr->set(i_c);
			i_meas += i_c;

			tracer.trace();
		}
		i_meas = i_meas / 100.00;
		measIShPtr->set(i_meas);

	}




}