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

#pragma once

#include "SineWaveGenerator.h"
#include <tuple>
#include <vector>
#include <utility>
#include <cfloat> 
#include <functional> 
#include <optional>


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
            

            struct ResultFrequencyMeasurement
            {
                std::double_t frequency = 0.00;    // Frequency in Hz               
                std::double_t magnitude = 0.00;    // Magnitude (normalized value)
                std::double_t phase     = 0.00;    // Phase in Degrees
                explicit ResultFrequencyMeasurement(std::double_t freq, std::double_t mag, std::double_t ph)
                    :frequency(freq), magnitude(mag), phase(ph)
                {
                }
            };


            /**
             * @brief Construct a new Frequency Response Manager object.
             *
            
             * @param freqBands Vector of frequency bands for the response test.
             */
            FrequencyResponseManager(std::double_t sampling_period,
                const std::vector<FrequencyBand>& freqBands,
                std::function<std::double_t(std::double_t)> process_function,
                std::function<void(void)> reset_function);

            /**
             * @brief Get the total duration of all frequency sequences.
             *
             * @return std::double_t Total duration of the test sequences.
             */
            std::double_t getDuration() const;
                      
        
            bool process();

            inline size_t getNumberOfMeasurements() const
            {
                return _measurements.size();
            }

            inline std::double_t getFrequency(size_t index) const
            {
                if (index < _measurements.size())
                {
                    return _measurements[index].frequency;
                }
                return 0.00;
            }

            inline double getMagnitude(size_t index) const
            {
                if (index < _measurements.size())
                {
                    return _measurements[index].magnitude;
                }
                return 0.00;
            }

            inline double getPhase(size_t index) const
            {
                if (index < _measurements.size())
                {
                    return _measurements[index].phase;
                }
                return 0.00;
            }

            
            std::optional<ResultFrequencyMeasurement> findMeasurementByFrequency( std::double_t targetFrequency, std::double_t tolerance ) const;
           

        private:

            std::function<std::double_t(std::double_t)> _process_function;
            std::function<void(void)> _reset_function;

            SineWaveGenerator _generator;

            std::double_t _samplingPeriod;
            std::vector<FrequencyBand> _freq_bands;
                      

            std::double_t _totalDuration;
         
            std::vector<std::pair<std::double_t, std::double_t>> _toMeasure;

            std::vector <ResultFrequencyMeasurement> _measurements;

        };
    }
}