#include "pch.h"
#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <vector>
#include "StringUtil.h"
#include "WaveFormTracer.h"
#include "SOSystem.h"
#include "FrequencyResponseManager.h"

#include <complex>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace CntrlLibrary;
using std::experimental::filesystem::path;

TEST(TestSecondOrderSystem, TestOsc)
{
	using namespace DiscreteTime;

	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/SecondOrderSystem.dat";

	std::double_t ts = 0.0001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto freqTp = tracer.addSignal<std::double_t>("frequency", BaseSignal::SignalType::Double);
	auto magTp = tracer.addSignal<std::double_t>("magnitude", BaseSignal::SignalType::Double);
	auto phaseTp = tracer.addSignal<std::double_t>("phase", BaseSignal::SignalType::Double);

	//define filter and main process function
	SOSystem sos;
	//100 Hz
	//fc = 100    # Cut - off frequency in Hz
	//zeta = 0.0  # Example damping ratio(no damping -oscilates at 100Hz)
	
	//sos.setParameters(-1.99605605, 1.00, 0.00098599, 0.00197197, 0.00098599, "SystemSecondOrder");
	// 

	std::complex<double> D (0.5,0.0);
	std::double_t w0 = 628.00;

	std::complex<double> pa1(0.0);
	std::complex<double> pa2(0.0);

	
		//the system is underdamped
	pa1 = -D * w0 + w0 * sqrt(D * D - 1.00);
	pa2 = -D * w0 - w0 * sqrt(D * D - 1.00);
	
	std::complex<double> p1 = exp(pa1 * (std::complex<double>) ts);
	std::complex<double> p2 = exp(pa2 * (std::complex<double>) ts);

	std::double_t b0 = 1.0;
	std::double_t b1 = 2.0;
	std::double_t b2 = 1.0;

	std::double_t a1 = - (std::abs(p1) + std::abs(p2));
	std::double_t a2 = std::abs(p1) * std::abs(p2);

	std::double_t k = abs((1.0 + a1 + a2) / (b0 + b1 + b2));

	std::double_t diff = abs(a1) - abs(a2) - 1.00;
	a1 += 2*diff; //ensure stability!!!???
	a2 += diff; //ensure stability!!!???
	


	sos.setParameters(k, a1, a2, b0, b1, b2, "SystemSecondOrder");

	auto procFunc = [&sos](double val) -> double
	{
		return sos.process(val);
	};

	auto resetFunc = [&sos](void) -> void
	{
		return sos.reset();
	};

	std::vector<FrequencyResponseManager::FrequencyBand> frequencyBands;
	std::double_t minFrequency = 10;
	frequencyBands.emplace_back(minFrequency, 500, 1000, 1.00);
	FrequencyResponseManager frManager(ts, frequencyBands, procFunc, resetFunc);
	std::double_t fo(0.00);
	std::vector<std::double_t> measurements;
	std::double_t ttime = frManager.getDuration();
	EXPECT_TRUE(frManager.process());

	std::double_t fr(0.00);
	size_t nMeas = frManager.getNumberOfMeasurements();
	for (size_t i = 0; i < nMeas; i++)
	{
		fr = frManager.getFrequency(i);
		freqTp->set(fr);
		magTp->set(20.0 * log10(frManager.getMagnitude(i)));
		phaseTp->set(frManager.getPhase(i));
		tracer.trace();
	}
}
