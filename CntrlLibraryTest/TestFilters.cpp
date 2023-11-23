#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <vector>
#include "StringUtil.h"
#include "WaveFormTracer.h"
#include "IIRFilterButterFO.h"
#include "IIRFilterButterSO.h"
#include "FrequencyResponseManager.h"
#include "FFTUtility.h"
#include "BasicNumMethods.h"
#include "window.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace CntrlLibrary;
using std::experimental::filesystem::path;

TEST(TestButterworthLowPassI, TestLowPass1)
{
	using namespace DiscreteTime;
	using namespace Filters;
	using namespace Math;


	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestLowPass1.dat";

	std::double_t ts = 0.0001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto freqTp = tracer.addSignal<std::double_t>("frequency", BaseSignal::SignalType::Double);
	auto magTp = tracer.addSignal<std::double_t>("magnitude", BaseSignal::SignalType::Double);
	auto phaseTp = tracer.addSignal<std::double_t>("phase", BaseSignal::SignalType::Double);
	
	//define filter and main process function
	ButterworthLowPassI filter;
	std::double_t filt_fc = 100; //100 Hz
	filter.setCutoffFrequency(filt_fc * 2.00 * M_PI , ts, "TestLowPassB1"); //100 Hz
	auto procFunc = [&filter](double val) -> double
	{
		return filter.process(val);
	};

	std::vector<FrequencyResponseManager::FrequencyBand> frequencyBands;
	std::double_t minFrequency = 10;
	frequencyBands.emplace_back(minFrequency, 505, 1000, 1.00);
	FrequencyResponseManager frManager(  ts, frequencyBands, procFunc);
	std::double_t fo(0.00);
	std::vector<std::double_t> measurements;
	std::double_t ttime = frManager.getDuration();
	EXPECT_TRUE(frManager.process());

	auto optResult = frManager.findMeasurementByFrequency(filt_fc, 1);
	EXPECT_TRUE(optResult != std::nullopt);
	EXPECT_NEAR(0.71, optResult->magnitude, 0.1 ); //-3dB at Fc
	
	std::double_t fr(0.00);
	size_t nMeas = frManager.getNumberOfMeasurements();
	for (size_t i = 0; i < nMeas; i++)
	{
		fr = frManager.getFrequency(i);
		freqTp->set(fr);
		magTp->set(  20.00 + 20.0 * log10( frManager.getMagnitude(i) ));
		phaseTp->set(frManager.getPhase(i));		
		tracer.trace();
	}
 }


TEST(TestButterworthLowPassII, TestLowPass2)
{
	using namespace DiscreteTime;
	using namespace Filters;
	using namespace Math;


	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestLowPass2.dat";

	std::double_t ts = 0.0001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto freqTp = tracer.addSignal<std::double_t>("frequency", BaseSignal::SignalType::Double);
	auto magTp = tracer.addSignal<std::double_t>("magnitude", BaseSignal::SignalType::Double);
	auto phaseTp = tracer.addSignal<std::double_t>("phase", BaseSignal::SignalType::Double);

	//define filter and main process function
	ButterworthLowPassII filter;
	std::double_t filt_fc = 100; //100 Hz
	filter.setCutoffFrequency(filt_fc * 2.00 * M_PI, ts, "TestLowPassB2"); //100 Hz
	auto procFunc = [&filter](double val) -> double
	{
		return filter.process(val);
	};

	std::vector<FrequencyResponseManager::FrequencyBand> frequencyBands;
	std::double_t minFrequency = 10;
	frequencyBands.emplace_back(minFrequency, 505, 1000, 1.00);
	FrequencyResponseManager frManager(ts, frequencyBands, procFunc);
	std::double_t fo(0.00);
	std::vector<std::double_t> measurements;
	std::double_t ttime = frManager.getDuration();
	EXPECT_TRUE(frManager.process());
	 
	auto optResult = frManager.findMeasurementByFrequency(filt_fc, 1);
	EXPECT_TRUE(optResult != std::nullopt);
	EXPECT_NEAR(0.71, optResult->magnitude, 0.1); //-3dB at Fc

	std::double_t fr(0.00);
	size_t nMeas = frManager.getNumberOfMeasurements();
	for (size_t i = 0; i < nMeas; i++)
	{
		fr = frManager.getFrequency(i);
		freqTp->set(fr);
		magTp->set(20.00 + 20.0 * log10(frManager.getMagnitude(i)));
		phaseTp->set(frManager.getPhase(i));
		tracer.trace();
	}
}


TEST(TestButterworthHighPassII, TestHighPass2)
{
	using namespace DiscreteTime;
	using namespace Filters;
	using namespace Math;


	auto path = std::filesystem::current_path();

	std::string strpath = path.generic_string();
	StringUtil::remove_substring(strpath, "CntrlLibraryTest");
	std::string fileName1 = strpath + "test/TestHighPass2.dat";

	std::double_t ts = 0.0001;
	WaveFormTracer tracer(fileName1, ts);
	EXPECT_TRUE(tracer.open());

	auto freqTp = tracer.addSignal<std::double_t>("frequency", BaseSignal::SignalType::Double);
	auto magTp = tracer.addSignal<std::double_t>("magnitude", BaseSignal::SignalType::Double);
	auto phaseTp = tracer.addSignal<std::double_t>("phase", BaseSignal::SignalType::Double);

	//define filter and main process function
	ButterworthHighPassII filter;
	std::double_t filt_fc = 100; //100 Hz
	filter.setCutoffFrequency(filt_fc * 2.00 * M_PI, ts, "TestLowPassB2"); //100 Hz
	auto procFunc = [&filter](double val) -> double
	{
		return filter.process(val);
	};

	std::vector<FrequencyResponseManager::FrequencyBand> frequencyBands;
	std::double_t minFrequency = 10;
	frequencyBands.emplace_back(minFrequency, 505, 1000, 1.00);
	FrequencyResponseManager frManager(ts, frequencyBands, procFunc);
	std::double_t fo(0.00);
	std::vector<std::double_t> measurements;
	std::double_t ttime = frManager.getDuration();
	EXPECT_TRUE(frManager.process());

	auto optResult = frManager.findMeasurementByFrequency(filt_fc, 1);
	EXPECT_TRUE(optResult != std::nullopt);
	EXPECT_NEAR(0.71, optResult->magnitude, 0.1); //-3dB at Fc

	std::double_t fr(0.00);
	size_t nMeas = frManager.getNumberOfMeasurements();
	for (size_t i = 0; i < nMeas; i++)
	{
		fr = frManager.getFrequency(i);
		freqTp->set(fr);
		magTp->set(20.00 + 20.0 * log10(frManager.getMagnitude(i)));
		phaseTp->set(frManager.getPhase(i));
		tracer.trace();
	}
}


/*


//size_t num = 2 * (size_t) (ttime / ts);
	//std::double_t samplingRate = 1.00 / ts;

	/*
	//round to next pow2
	size_t oldNum = num;

	num = (size_t)BasicNumMethods::Pow2Util::nextPow2((uint32_t)num);
	measurements.reserve(num);
	size_t index = 0U;


	std::unique_ptr<BaseWindow> ptrWindow = WindowFactory::create(BaseWindow::Window::Hamming, num, 1.00);
	int numOfCoefs = ptrWindow->length();
	double nominator(0.0), denominator(0.0);

	for (int i = 0; i < numOfCoefs; i++)
	{
		double omega = ptrWindow->w(i);
		nominator += omega * omega;
		denominator += omega;
	}
	double normEffNoiseBandwith(1.0); //The normalized effective noise bandwidt
	if (denominator > 0)
	{
		normEffNoiseBandwith = numOfCoefs * nominator / (denominator * denominator);
	}

	for (size_t n = index; n < num; n++)
	{
		if (n >= measurements.size())
		{
			measurements.resize(n+1);
		}
		measurements[n] = 0.00;
	}

	FFTUtility fft;
	fft.fft(measurements);
	std::double_t val;
	std::double_t mag;
	std::double_t freq;

	std::vector<std::double_t> fre;

	 for( unsigned int k = 1; k < num/2 - 1; k++ )
	 {
		val = FFTUtility::fftMagdB( measurements, num/2, k, 1.0 );
		freq =  FFTUtility::fftFrequency (num/2, k, samplingRate  );
		fre.push_back(freq);
		sin->set(val);
		filt->set(freq);
		tracer.trace();

	 }

*/