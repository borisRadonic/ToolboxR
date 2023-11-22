#include "FrequencyResponseManager.h"
#define _USE_MATH_DEFINES
#include <math.h>

namespace CntrlLibrary
{
    namespace DiscreteTime
    {
        FrequencyResponseManager::FrequencyResponseManager(std::double_t sampling_period,
                                                           const std::vector<FrequencyBand>& freqBands,
                                                           std::function<std::double_t(std::double_t)> function)
            :_samplingPeriod(sampling_period)
            , _freq_bands(freqBands)
            ,_totalDuration(0.0)
            ,_process_function(function)
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
                }
                double phase = -(360.00 * ((max_time_pr - max_time) / dT));
                rms_pr = sqrt(rms_pr / samples);
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