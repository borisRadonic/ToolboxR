#include "pch.h"

#include <iostream>
#include <memory>
#include <filesystem>
#include <sstream>
#include <vector>
#include "StringUtil.h"
#include "WaveFormTracer.h"
#include "IIRFilterButterFO.h"
#include "FrequencyResponseManager.h"
#include "FFTUtility.h"
#include "BasicNumMethods.h"

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

	auto sin = tracer.addSignal<std::double_t>("sin", BaseSignal::SignalType::Double);
	auto filt = tracer.addSignal<std::double_t>("filt", BaseSignal::SignalType::Double);
	

	ButterworthLowPassI filter;
	
	filter.setCutoffFrequency(628, ts, "TestLowPassB1");
	FrequencyResponseManager::ProcessResult result;
	result.isFinished = false;

	std::vector<FrequencyResponseManager::FrequencyBand> frequencyBands;
	frequencyBands.emplace_back(10, 1000, 1024, 1.00);
	//frequencyBands.emplace_back(100, 1000, 100, 1.00);
	//frequencyBands.emplace_back(1000, 3000, 30, 1.00);		
	FrequencyResponseManager frManager(  ts, frequencyBands);
	std::double_t fo(0.00);
	std::vector<std::double_t> measurements;
	std::double_t ttime = frManager.getDuration();
	size_t num = 2 * (size_t) (ttime / ts);  

	std::double_t samplingRate = 1.00 / ts;
	//round to next pow2
	num = (size_t)BasicNumMethods::Pow2Util::nextPow2((uint32_t)num);
	measurements.reserve(num);
	size_t index = 0U;
	while (!result.isFinished)
	{
		frManager.process(result);
				
		fo = filter.process(result.sineValue);
		/*
		sin->set(result.sineValue);
		filt->set(fo);
		tracer.trace();
		*/
		if((index *2+1) >= measurements.size())
		{
			measurements.resize((2 * index + 2));
		}
		measurements[index * 2] = fo;
		measurements[index * 2 + 1] = 0.00; //imaginary part
		index++;
	}
	//add 0.00 to the rest of data
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
        val = FFTUtility::fftMagdB( measurements, num/4, k, 1.0 );
        //mag = FFTUtility::fftMagnitude(measurements, num/2, k );          
        freq =  FFTUtility::fftFrequency (num/4, k, samplingRate);
		fre.push_back(freq);
		sin->set(val);
		filt->set(freq);



		tracer.trace();
            //xValues.push_back( freq );
            //yValues.push_back( average );
            //magValues.push_back( averageMag );
     }




	//double a = fft.fftMagnitude(measurements, unsigned long totalPoints, unsigned long frequencyIndex)
}