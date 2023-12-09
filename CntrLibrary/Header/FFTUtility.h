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

