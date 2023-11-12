#pragma once
#include <functional> 
#include <cmath>
#include <limits>


namespace CntrlLibrary
{
    namespace Math
    {

        // derived from Numerical Recipes in C, Cambridge University Press

        class FFTUtility
        {
        public:

            static void fft(std::vector<double>& data);
            
            static double fftMagnitude(const std::vector<double>& data, unsigned long totalPoints, unsigned long frequencyIndex);
            
            static double fftMagdB(const std::vector<double>& data, unsigned long totalPoints, unsigned long frequencyIndex, double fullScaleVoltage);
            
            static double fftPhase(const std::vector<double>& data, unsigned long totalPoints, unsigned long frequencyIndex);
            
            static double fftFrequency(unsigned long totalPoints, unsigned long frequencyIndex, double samplingRate);
            
            static double goertzel(unsigned int numSamples, unsigned int targetFrequency, unsigned int sampleRate, const std::vector<double>& data);
        };
    }
}

