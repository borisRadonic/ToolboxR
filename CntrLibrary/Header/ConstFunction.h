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
#include "MathFunctionBase.h"

namespace CntrlLibrary
{
    namespace Math
    {
        class ConstFunction : public MathFunctionBase
        {
        

        public:
            ConstFunction(double constantValue) : _c(constantValue)
            {
            }

            inline virtual double compute(double t) override
            {
                return _c; //// Constant value regardless of t
            }

            inline virtual double firstDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double secondDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double thirdDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double fourthDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double firstIntegral(double t, double c1 = 0.00 ) override
            {
                return(_c * t + c1);
            }

            inline virtual double secondIntegral(double t, double c1 = 0.00, double c2 = 0.00) override
            {
                return( 0.5 * _c * t * t + c1 *t + c2);
            }

        private:
            double _c; // Constant value
        };
    }
}
