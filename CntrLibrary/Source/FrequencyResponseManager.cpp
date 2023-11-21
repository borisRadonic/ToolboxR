#include "FrequencyResponseManager.h"

namespace CntrlLibrary
{
    namespace DiscreteTime
    {
        FrequencyResponseManager::FrequencyResponseManager(std::double_t sampling_period,
                                                           const std::vector<FrequencyBand>& freqBands,
                                                           std::function<std::double_t(std::double_t)> function)
            :_samplingPeriod(sampling_period)
            , _freq_bands(freqBands)
            ,_currentFrequencyIndex(0)
            ,_currentTime(0.0)
            ,_totalDuration(0.0)
            , _totTime(0.00)
            ,_finished(false)
            ,_nextSequenceTime(0.00)
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
            updateFrequency();
        }

        std::double_t FrequencyResponseManager::getDuration() const
        {
            return _totalDuration;
        }

        bool FrequencyResponseManager::process()
        {
            if (_finished)
            {               
                return true;
            }

            if (_currentFrequencyIndex > _toMeasure.size())
            {
                return true;
            }
            double sineValue = _generator.process(_currentTime);
            double resValue = _process_function(sineValue);

            _currentTime += _samplingPeriod;

            _totTime += _samplingPeriod;
            
            double amplitude = _toMeasure[_currentFrequencyIndex].second;
            double freq = _toMeasure[_currentFrequencyIndex].first;
                        

            if (_currentTime >= _nextSequenceTime)
            {
                updateFrequency();
            }

            _finished = (_currentFrequencyIndex == 0 && _totTime >= _totalDuration);
        }

        void FrequencyResponseManager::updateFrequency()
        {
            if (_currentFrequencyIndex < _toMeasure.size() - 1)
            {
                _currentFrequencyIndex++;
            }
            else
            {
                // Resets to the first frequency after the last one
                _currentFrequencyIndex = 0;
            }
            _generator.setParameters(_toMeasure[_currentFrequencyIndex].second, _toMeasure[_currentFrequencyIndex].first, 0.00, "");
            _nextSequenceTime = (1.00 / _toMeasure[_currentFrequencyIndex].first);
            _currentTime = 0.00;
        }
                
    }
}