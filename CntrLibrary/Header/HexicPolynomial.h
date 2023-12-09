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

#ifndef NO_EXCEPTION
#include<stdexcept>
#endif

namespace CntrlLibrary
{
    namespace Math
    {
        namespace Polynomials
        {
            class HexicPolynomial : public MathFunctionBase
            {
            public:

                HexicPolynomial()
                {
                }

                inline double compute(double t)
                {
                    double t_2 = t * t;
                    double t_3 = t_2 * t;
                    double t_4 = t_3 * t;
                    double t_5 = t_4 * t;
                    double t_6 = t_5 * t;

                    return(_a0 + _a1 * t + _a2 * t_2 + _a3 * t_3 + _a4 * t_4 + _a5 * t_5 + _a6 * t_6);                   
                }

                inline double firstDerivative(double t) override
                {
                    double t_2 = t * t;
                    double t_3 = t_2 * t;
                    double t_4 = t_3 * t;
                    double t_5 = t_4 * t;

                    return (_a1 + 2.00 * _a2 * t + 3.00 * _a3 * t_2 + 4.00 * _a4 * t_3 + 5.00 * _a5 * t_4 + 6.00 * _a6 * t_5 );
                }

                inline double secondDerivative(double t) override
                {
                    double t_2 = t * t;
                    double t_3 = t_2 * t;
                    double t_4 = t_3 * t;
   
                    return (2.00 * _a2 + 6.00 * _a3 * t + 12.00 * _a4 * t_2 + 20.00 * _a5 * t_3 + 30.00 * _a6 * t_4 );
                }

                inline double thirdDerivative(double t) override
                {
                    double t_2 = t * t;
                    double t_3 = t_2 * t;    
                    return (6.00 * _a3 + 24.00 * _a4 * t + 60.00 * _a5 * t_2 + 120.00 * _a6 * t_3 );
                }

                inline double fourthDerivative(double t) override
                {
                    double t_2 = t * t; 
                    return (24.00 * _a4 + 120.00 * _a5 * t + 360.00 * _a6 * t_2 );
                }


                virtual double firstIntegral(double t, double c1) override
                {
                    #ifndef NO_EXCEPTION
                        throw std::runtime_error("fourthIntegral not implemented");
                    #else
                        return 0.00;
                    #endif                    
                    }

                    virtual double secondIntegral(double t, double c1, double c2) override
                    {
                        #ifndef NO_EXCEPTION
                            throw std::runtime_error("fourthIntegral not implemented");
                        #else
                            return 0.00;
                        #endif
                }

                inline void calculate(double t, double& x, double& x_der1, double& x_der2, double& x_der3, double& x_der4)
                {
                    double t_2 = t * t;
                    double t_3 = t_2 * t;
                    double t_4 = t_3 * t;
                    double t_5 = t_4 * t;
                    double t_6 = t_5 * t;

                    x = _a0 + _a1 * t + _a2 * t_2 + _a3 * t_3 + _a4 * t_4 + _a5 * t_5 + _a6 * t_6;
                    x_der1 = _a1 + 2.00 * _a2 * t + 3.00 * _a3 * t_2 + 4.00 * _a4 * t_3 + 5.00 * _a5 * t_4 + 6.00 * _a6 * t_5;
                    x_der2 = 2.00 * _a2 + 6.00 * _a3 * t + 12.00 * _a4 * t_2 + 20.00 * _a5 * t_3 + 30.00 * _a6 * t_4;
                    x_der3 = 6.00 * _a3 + 24.00 * _a4 * t + 60.00 * _a5 * t_2 + 120.00 * _a6 * t_3;
                    x_der4 = 24.00 * _a4  + 120.00 * _a5 * t + 360.00 * _a6 * t_2;
                }

                inline void setParams(double a0, double a1, double a2, double a3, double a4, double a5, double a6)
                {
                    _a0 = a0;
                    _a1 = a1;
                    _a2 = a2;
                    _a3 = a3;
                    _a4 = a4;
                    _a5 = a5;
                    _a6 = a6;
                }

                inline double getA0() const
                {
                    return _a0;
                }

                inline double getA1() const
                {
                    return _a1;
                }

                inline double getA2() const
                {
                    return _a2;
                }

                inline double getA3() const
                {
                    return _a3;
                }

                inline double getA4() const
                {
                    return _a4;
                }

                inline double getA5() const
                {
                    return _a5;
                }

                inline double getA6() const
                {
                    return _a6;
                }

            private:

                double _a0 = 0.00;
                double _a1 = 0.00;
                double _a2 = 0.00;
                double _a3 = 0.00;
                double _a4 = 0.00;
                double _a5 = 0.00;
                double _a6 = 0.00;
            };
        }
    }
}
