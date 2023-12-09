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
/*
#include <functional> 
#include <cmath>
#include <limits>
#include "MathFunctionBase.h"
#include "QuinticPolynomial.h"

#ifndef NO_EXCEPTION
#include<stdexcept>
#endif

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {        
        using namespace Math::Polynomials;

        class QuinticPolynomialOptimizer
        {           

        public:

            QuinticPolynomialOptimizer(QuinticPolynomial& poly, double max_vel, double max_acc, double max_jerk)
                : _poly(poly), _max_vel(max_vel), _max_acc(max_acc), _max_jerk(max_jerk)
            {
            }

            double objectiveFunction(double t)
            {
                double penalty = 0.0;
                double dt = t / static_cast<double>(_num_samples);

                for (int i = 0; i <= _num_samples; ++i)
                {
                    double current_t = dt * i;
                    double vel = _poly.firstDerivative(current_t);
                    double acc = _poly.secondDerivative(current_t);
                    double jerk = _poly.thirdDerivative(current_t);

                    if (std::abs(vel) > _max_vel)
                    {
                        penalty += _vel_penalty_factor * std::pow(std::abs(vel) - _max_vel, 2.00);
                    }
                    if (std::abs(acc) > _max_acc)
                    {
                        penalty += _acc_penalty_factor * std::pow(std::abs(acc) - _max_acc, 2.00);
                    }
                    if (std::abs(jerk) > _max_jerk)
                    {
                        penalty += _jerk_penalty_factor * std::pow(std::abs(jerk) - _max_jerk, 2.00);
                    }
                }

                return t + penalty; // The objective is to minimize time t, increased by any penalties for constraint violations.
            }

            double optimize()
            {
                //TODO....
                return 0.00
            }


        private:

            QuinticPolynomial& _poly;
            double _max_vel;
            double _max_acc;
            double _max_jerk;
            int _num_samples = 100; // Number of samples to check along the trajectory for constraint violations
            double _vel_penalty_factor = 1000.0;
            double _acc_penalty_factor = 1000.0;
            double _jerk_penalty_factor = 100.0;
        };
    }
}
*/
