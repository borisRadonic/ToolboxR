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
        namespace BasicNumMethods
        {

            class Pow2Util
            {
            public:

                static uint32_t nextPow2(uint32_t value)
                {
                    if (value == 0)
                    {
                        return 1U;
                    }

                    // Decrement the value
                    value--;

                    // Set all bits below the highest set bit
                    value |= value >> 1;
                    value |= value >> 2;
                    value |= value >> 4;
                    value |= value >> 8;
                    value |= value >> 16;

                    // Increment to get the next power of 2
                    return value + 1U;
                }
            };



            enum class ResultType
            {
                Ok = 0,
                WrongInputParameters = 1,
                NotPossible = 3,
                NegativeTime = 4,
                OutOfTolerance = 5
            };

            class NewtonRaphson
            {
            public:

                NewtonRaphson() = delete;

                explicit NewtonRaphson(std::function<double(double)> function, std::function<double(double)> derivative, double tolerance = 1e-6, int maxIterations = 1000)
                    : _func(function), _dfunc(derivative), _tol(tolerance), _max_iter(maxIterations)
                {
                }

                /*only for x > 0*/
                ResultType findRoot(double initial_guess, double& root )
                {
                    if (initial_guess < 0.00)
                    {
                        root = 0.00;
                        return ResultType::WrongInputParameters;
                    }
                    double x = initial_guess;
                    for (int i = 0; i < _max_iter; ++i)
                    {
                        double df = _dfunc(x);
                        if (abs(df) > _tol)
                        {
                            double f = _func(x);
                            x = x - f/ df;
                            if (x < 0.00)
                            {
                                return ResultType::NegativeTime;
                            }
                           
                            if (std::abs(f) < _tol)
                            {
                                root = x;
                                return ResultType::Ok;
                            }
                        }
                        else
                        {
                            /*it is maximum*/
                            root = x;
                            return ResultType::Ok;
                        }
                    }
                    root = x;  // the last computed value
                    return ResultType::OutOfTolerance;
                }

            private:
                std::function<double(double)> _func;   // Function for which root is to be found
                std::function<double(double)> _dfunc;  // Derivative of the function
                double _tol;
                int _max_iter;
            };


            class SimpsonsIntegrator
            {
            public:
                static ResultType approximateIntegral(std::function<double(double)> f, double x1, double x2, int n, double& outResult)
                {
                    if (n % 2 != 0)
                    {
                        return ResultType::WrongInputParameters;
                    }
                    double h = (x2 - x1) / n;
                    // Initialize with the boundary values
                    double integral = f(x1) + f(x2);

                    for (int i = 1; i < n; i += 2)
                    {
                        integral += 4 * f(x1 + i * h);
                    }

                    for (int i = 2; i < n - 1; i += 2)
                    {
                        integral += 2 * f(x1 + i * h);
                    }
                    if (h > std::numeric_limits<double>::min())
                    {
                        integral *= h / 3.0;
                    }
                    else
                    {
                        return ResultType::NotPossible;
                    }
                    outResult = integral;
                    return ResultType::Ok;
                }
            };
        }
    }
}


