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
#include "BasicNumMethods.h"
#include <cmath>
#include <algorithm>
#include "HepticPolynomial.h"

using namespace CntrlLibrary::Math::BasicNumMethods;
using namespace CntrlLibrary::Math::Polynomials;


namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {
        /*very simple trajectory generation Initial and final jerks, velocities and accelerations must be 0*/

        class SimplelHepticTrajectory
        {
        public:
            SimplelHepticTrajectory() = delete;

            /**
            * @brief Constructs a Quintic Polynomial Trajectory.
            *
            * Initializes a trajectory using a quintic polynomial based on the provided initial
            * and final conditions for position, velocity, and acceleration.
            *
            * @param i_pos    Initial position value for the trajectory.
            * @param f_pos    Final position value for the trajectory.           
            * @param f_time   Time duration for which the trajectory is defined.
            */
            explicit SimplelHepticTrajectory(double i_pos, double f_pos, double v_max, double a_max, double f_time);

          
            double calculateMinTime(double vel_tolerance, double accel_tolerance);

            void create(double f_time);


            /**
             * @brief Calculate position, velocity, acceleration, and jerk based on the input time.
             *
             * Given a time value @p t, this function computes and updates the associated values of
             * position, velocity, acceleration, and jerk.
             *
             * @param t            The input time value.
             * @param[out] position     Calculated position at time @p t.
             * @param[out] velocity     Calculated velocity at time @p t.
             * @param[out] acceleration Calculated acceleration at time @p t.
             * @param[out] jerk         Calculated jerk at time @p t.
             */
            void calculateValuesForTime(double t, double& position, double& velocity, double& acceleration, double& jerk);

            inline const HepticPolynomial& getPoly() const
            {
                return poly;
            }

        private:

            double _final_time = 0.00001;

            double max_acceleration = 0.00;
            double max_velocity = 0.00;

            double initial_position = 0.00;
            double target_position = 0.00;
                      
            HepticPolynomial poly;


        };
    }
}
