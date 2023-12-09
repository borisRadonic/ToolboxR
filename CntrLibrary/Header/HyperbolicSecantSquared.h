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
        class HyperbolicSecantSquared : public MathFunctionBase
        {
       
        public:

            HyperbolicSecantSquared(double a):_a(a)
            {
            }

            inline virtual double compute(double t) override
            {
                 return _a * std::pow(sech(t), 2.00);
            }

            inline virtual double firstDerivative(double t) override
            {
                return ( - 2.00 * std::pow(sech(t), 2.00) * std::tanh(t) );
            }

            inline virtual double secondDerivative(double t) override
            {
                return (-4.00 * std::pow(sech(t), 2.00) * std::pow(tanh(t), 2.00) - 2.00 * std::pow(sech(t), 4.00));;
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

            inline virtual double firstIntegral(double t, double c1 = 0.00) override
            {
                return(std::tanh(t) + c1);
            }

            inline virtual double secondIntegral(double t, double c1 = 0.00, double c2 = 0.00) override
            {
                return(_a * std::log( abs( cosh(t) ) ) + c1 * t + c2);
            }

        private:
            double sech(double x) const
            {
                // Hyperbolic secant is 1 / cosh(x)
                return 1.0 / std::cosh(x);
            }

        private:
            double _a; //amplitude
        };
    }
}

