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

#include "FrequencyResponseManager.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
    namespace DiscreteTime
    {
        FrequencyResponseManager::FrequencyResponseManager(std::double_t sampling_period,
                                                           const std::vector<FrequencyBand>& freqBands,
                                                           std::function<std::double_t(std::double_t)> process_function,
                                                           std::function<void(void)> reset_function)
            :_samplingPeriod(sampling_period)
            , _freq_bands(freqBands)
            ,_totalDuration(0.0)
            ,_process_function(process_function)
            ,_reset_function(reset_function)
        {
            for (auto band : _freq_bands)
            {
                if ((band.endFrequency > band.startFrequency) && (band.numberOfMeasurements > 0) )
                {
                    std::double_t fband = band.endFrequency - band.startFrequency;
                    std::double_t df = fband / band.numberOfMeasurements;
                    std::double_t f = 0.00;
                    for (int i = 0; i < band.numberOfMeasurements; i++)
                    {
                        f = band.startFrequency + i * df;
                        _totalDuration += 1.00 / f;      
                        _toMeasure.emplace_back( f, band.signalAmplitude );
                    }
                }
            }            
        }

        std::double_t FrequencyResponseManager::getDuration() const
        {
            return _totalDuration;
        }

        bool FrequencyResponseManager::process()
        {
            bool isMax(false);
            std::double_t amplitude(0.00);
            std::double_t freq(0.00);
            std::double_t sineValue(0.00);
            std::double_t resValue(0.00);
            std::double_t max_time(0.00);
            std::double_t dT(0.00);
            std::double_t currentTime;
            std::double_t max_val_pr(0.00);
            std::double_t max_time_pr(0.00);
            std::double_t rms_pr(0.00);
            std::double_t samples(0.00);

            for(auto meas: _toMeasure)
            {
                currentTime = 0.00;
                amplitude = meas.second;
                freq = meas.first;
                dT = 1.00 / freq;
                rms_pr = 0.00;
                max_val_pr = 0.00;
                max_time_pr = 0.00;
                _generator.setParameters(amplitude, freq, 0.00, _samplingPeriod, "");
                samples = dT / _samplingPeriod;
                int smpl = 0;
                _reset_function();
                while (currentTime < dT)
                {
                    sineValue = _generator.process(currentTime, isMax);
                    resValue = _process_function(sineValue);
                    rms_pr += pow(abs(resValue), 2);
                    if (resValue > max_val_pr)
                    {
                        max_val_pr = resValue;
                        max_time_pr = currentTime;
                    }
                    if (isMax)
                    {
                        max_time = currentTime;
                    }
                    currentTime += _samplingPeriod;
                    smpl++;
                }
                double phase = -(360.00 * ((max_time_pr - max_time) / dT));
                rms_pr = sqrt(rms_pr / (double)smpl);
                _measurements.emplace_back( freq, rms_pr / (amplitude * M_SQRT1_2), phase );
            }
            return true;
        }
        std::optional<FrequencyResponseManager::ResultFrequencyMeasurement> FrequencyResponseManager::findMeasurementByFrequency(double targetFrequency, std::double_t tolerance ) const
        {
            for (const auto& measurement : _measurements)
            {
                if (std::abs(measurement.frequency - targetFrequency) <= tolerance)
                {
                    return measurement;
                }
            }
            return std::nullopt; // Return an empty optional if not found
        }
    }
}