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
#include <cmath>
#include <limits>
#include "MathFunctionBase.h"

#ifndef NO_EXCEPTION
#include<stdexcept>
#endif

namespace CntrlLibrary
{
    namespace Math
    {
        class LinearExponentialDecay : public MathFunctionBase
        {

        public:

            explicit LinearExponentialDecay(double decayRate)
                : _k(decayRate)
            {
            }

            inline virtual double compute(double t) override
            {
                return ( (1.00 - t) * std::exp(-_k * t) );
            }

            // First derivative: f'(x) = -e^(-kx) + kx * e^(-kx)
            inline virtual double firstDerivative(double t) override
            {
                return ( (-1.00 + _k * t) * std::exp(-_k * t) );
            }

            inline virtual double secondDerivative(double t) override
            {
                return ( (_k * _k * t - 2.00 * _k) * std::exp(-_k * t) );
            }

            inline virtual double thirdDerivative(double /* t */) override
            {
#ifndef NO_EXCEPTION
                throw std::runtime_error("fourthIntegral not implemented");
#else
                return 0.00;
#endif
            }

            inline virtual double fourthDerivative(double /* t */) override
            {
#ifndef NO_EXCEPTION
                throw std::runtime_error("fourthIntegral not implemented");
#else
                return 0.00;
#endif        
            }

            // First integral for k != 1: ∫(1 - x)e ^ (-kx)dx = (e ^ (-kx)(kx - 1) + 1) / k ^ 2
            inline virtual double firstIntegral(double t, double c1 = 0.00) override
            {
                if (_k == 1.0)
                {
                    #ifndef NO_EXCEPTION
                    throw std::invalid_argument("First integral not defined for k = 1.");
                    #else
                    return 0.00;
                    #endif 
                }
                return (std::exp(-_k * t) * (_k * t - 1.00) + 1.00) / (_k * _k);
            }

            inline virtual double secondIntegral(double t, double c1 = 0.00, double c2 = 0.00) override
            {
                //return(_a * std::log(abs(cosh(t))) + c1 * t + c2);
            }


        private:
            double _k; //Decay rate
        };
    }
}


