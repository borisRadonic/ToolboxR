
#pragma once

#include "SineWaveGenerator.h"
#include <tuple>
#include <vector>
#include <utility>
#include <cfloat> 
#include <functional> 


namespace CntrlLibrary
{
    namespace DiscreteTime
    {
        /**
       * @class FrequencyResponseManager
       * @brief Manages the generation of frequency response stimuli.
       *
       * This class is responsible for generating a series of sine wave signals
       * across different frequencies, typically used for frequency response testing.
       */
        class FrequencyResponseManager
        {
        public:
                        

            struct FrequencyBand
            {
                std::double_t startFrequency;   // Start of the frequency band in Hz
                std::double_t endFrequency;     // End of the frequency band in Hz
                std::double_t signalAmplitude;  //  amplitude of test signal
                int numberOfMeasurements;       // Number of measurements within this frequency band

                explicit FrequencyBand(std::double_t start, std::double_t end, int numMeasurements, std::double_t amplitude)
                    : startFrequency(start)
                    , endFrequency(end)
                    , signalAmplitude(amplitude)
                    , numberOfMeasurements(numMeasurements)
                {
                }
            };
                        

            /**
             * @brief Construct a new Frequency Response Manager object.
             *
            
             * @param freqBands Vector of frequency bands for the response test.
             */
            FrequencyResponseManager(std::double_t sampling_period, const std::vector<FrequencyBand>& freqBands, std::function<std::double_t(std::double_t)> function);

            /**
             * @brief Get the total duration of all frequency sequences.
             *
             * @return std::double_t Total duration of the test sequences.
             */
            std::double_t getDuration() const;
                      
        
            bool process();

        private:

            std::function<std::double_t(std::double_t)> _process_function;

            void updateFrequency();
                      
            SineWaveGenerator _generator;

            std::double_t _samplingPeriod;
            std::vector<FrequencyBand> _freq_bands;

            std::vector<std::pair<std::double_t, std::double_t>> _toMeasure;
         
            int _currentFrequencyIndex;
            std::double_t _currentTime;
            std::double_t _totTime;
            std::double_t _totalDuration;
            bool _finished;
            std::double_t _nextSequenceTime;  // Time when the next frequency sequence should start
        };
    }
}