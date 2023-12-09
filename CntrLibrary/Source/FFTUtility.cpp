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

#include "FFTUtility.h"
#include <numbers>
#include <assert.h>

namespace CntrlLibrary
{
    namespace Math
    {   

        // derived from Numerical Recipes in C, Cambridge University Press


        void FFTUtility::fft(std::vector<double>& data)
        {
            size_t numComplexElements = data.size() / 2;
            size_t n = numComplexElements << 1;
            size_t mmax(0U), istep(0U), m(0U), j(0U);

            // Bit-reversal reordering
            j = 1;
            for (size_t i = 1; i < n; i += 2)
            {
                if (j > i)
                {
                    std::swap(data[j - 1], data[i - 1]);
                    std::swap(data[j], data[i]);
                }
                m = n >> 1;
                while (m >= 2 && j > m)
                {
                    j -= m;
                    m >>= 1;
                }
                j += m;
            }

            // Danielson-Lanczos section
            for (mmax = 2; n > mmax; mmax <<= 1)
            {
                istep = mmax << 1;
                double theta = (-2 * std::numbers::pi) / mmax;
                double wtemp, wr, wpr, wpi, wi, tempr, tempi;

                wtemp = sin(0.5 * theta);
                wpr = -2.0 * wtemp * wtemp;
                wpi = sin(theta);
                wr = 1.0;
                wi = 0.0;

                for (m = 1; m < mmax; m += 2)
                {
                    for (size_t i = m; i <= n; i += istep)
                    {
                        j = i + mmax;
                        tempr = wr * data[j - 1] - wi * data[j];
                        tempi = wr * data[j] + wi * data[j - 1];

                        data[j - 1] = data[i - 1] - tempr;
                        data[j] = data[i] - tempi;
                        data[i - 1] += tempr;
                        data[i] += tempi;
                    }
                    wr = (wtemp = wr) * wpr - wi * wpi + wr;
                    wi = wi * wpr + wtemp * wpi + wi;
                }
            }
        }

        double FFTUtility::fftMagnitude(const std::vector<double>& data, unsigned long totalPoints, unsigned long frequencyIndex)
        {
            // Check if the frequency index is within the valid range
            if (frequencyIndex >= (totalPoints / 2) )
            {
                return 0.0; // Frequency index out of range
            }

            // Ensure the data array has enough elements
            if (data.size() < (2 * frequencyIndex + 1U))
            {
                return 0.0; // Data array does not have enough elements
            }

            // Calculate the real and imaginary parts of the FFT at the given frequency index
            double realComponent = fabs(data[2 * frequencyIndex]) + fabs(data[2 * totalPoints - 2 * frequencyIndex]);
            double imaginaryComponent = fabs(data[2 * frequencyIndex + 1]) + fabs(data[2 * totalPoints - 2 * frequencyIndex + 1]);

            // Compute and return the magnitude of the FFT at the given frequency index
            return sqrt(realComponent * realComponent + imaginaryComponent * imaginaryComponent) / totalPoints;
        }

        double FFTUtility::fftMagdB(const std::vector<double>& data, unsigned long totalPoints, unsigned long frequencyIndex, double fullScaleVoltage)
        {
            // Calculate the magnitude of the FFT at the given frequency index
            double fftMagnitudeValue = fftMagnitude(data, totalPoints, frequencyIndex);

            // Check if the magnitude is effectively zero to avoid logarithm of zero
            if (fftMagnitudeValue < std::numeric_limits<double>::min())
            {
                return -2000.0; // Return a very low dB value indicating out of range
            }

            // Calculate and return the magnitude in decibels (dB) relative to the full scale
            return 20.0 * log10(fftMagnitudeValue / fullScaleVoltage);
        }

        double FFTUtility::fftPhase(const std::vector<double>& data, unsigned long totalPoints, unsigned long frequencyIndex)
        {

            assert( data.size() < (2 * totalPoints) );
            if (data.size() < (2 * totalPoints))
            {
                return 0.0;
            }

            // Check if the frequency index is within the valid range
            if (frequencyIndex >= totalPoints / 2)
            {
                return 0.0; // Frequency index out of range
            }

            // Ensure the data array has enough elements
            if (data.size() < (2 * frequencyIndex + 1U))
            {
                return 0.0; // Data array does not have enough elements
            }

            // Handle the special case where the real part is zero
            if (data[2 * frequencyIndex] == 0.0)
            {
                // Return +/- 90 degrees based on the sign of the imaginary part
                return data[2 * frequencyIndex + 1] > 0.0 ? std::numbers::pi / 2 : -std::numbers::pi / 2;
            }

            // Calculate and return the phase angle in radians
            return atan2(data[2 * frequencyIndex + 1], data[2 * frequencyIndex]);
        }

        double FFTUtility::fftFrequency(unsigned long totalPoints, unsigned long frequencyIndex, double samplingRate)
        {

            // Check if the frequency index is within the valid range
            if (frequencyIndex >= totalPoints)
            {
                return 0.0; // Frequency index out of range
            }

            // Calculate the frequency for the given index
            if (frequencyIndex <= totalPoints / 2)
            {
                // For indices in the first half of the FFT spectrum
                return samplingRate * frequencyIndex / totalPoints;
            }
            else
            {
                // For indices in the second half of the FFT spectrum
                return -samplingRate * (totalPoints - frequencyIndex) / totalPoints;
            }
        }

        double FFTUtility::goertzel(unsigned int numSamples, unsigned int targetFrequency, unsigned int sampleRate, const std::vector<double>& data)
        {
            // Initialize variables for the Goertzel algorithm
            double q0 = 0.0, q1 = 0.0, q2 = 0.0;

            // Calculate the frequency bin 'k' for the target frequency
            unsigned int k = static_cast<unsigned int>(0.5 + ((numSamples * targetFrequency) / sampleRate));

            // Calculate the angular frequency
            double omega = (2.0 * std::numbers::pi * k) / numSamples;

            // Precompute sine and cosine values
            double sine = sin(omega), cosine = cos(omega);

            // Coefficient for the recursive relation
            double coeff = 2.0 * cosine;

            // Main loop of the Goertzel algorithm
            for (unsigned int i = 0; i < numSamples; i++) 
            {
                q0 = coeff * q1 - q2 + data[i];
                q2 = q1;
                q1 = q0;
            }

            // Calculate the real and imaginary parts of the result
            double realPart = (q1 - q2 * cosine);
            double imaginaryPart = (q2 * sine);

            // Return the power (squared magnitude) of the frequency component
            return (realPart * realPart + imaginaryPart * imaginaryPart);
        }
    }
}