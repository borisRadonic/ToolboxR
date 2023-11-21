
#pragma once

#include "SineWaveGenerator.h"
#include <tuple>
#include <vector>
#include <utility>
#include <cfloat> 


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

            struct ProcessResult
            {
                ProcessResult() :sineValue(0.00), currentFrequency(0.00), isFinished(false)
                {
                }
                std::double_t sineValue;         // The generated sine wave value
                std::double_t currentFrequency;  // The frequency in Hz being used in the current sequence                
                bool isFinished;                 // Flag to indicate if the frequency response test is complete
            };

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
            FrequencyResponseManager(std::double_t sampling_period, const std::vector<FrequencyBand>& freqBands);

            /**
             * @brief Get the total duration of all frequency sequences.
             *
             * @return std::double_t Total duration of the test sequences.
             */
            std::double_t getDuration() const;

           
            /**
            * @brief Processes the next step in the frequency response test and updates the result.
            *
            * This method updates the provided ProcessResult object with the current state of the
            * frequency response test, including the sine wave value, the current frequency,
            * the cycle number, and a flag indicating if the test is complete.
            *
            * @param result A reference to a ProcessResult object that will be updated with
            *               the current test results.
            */
            void process(ProcessResult& result);

        private:

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