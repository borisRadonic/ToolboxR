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

#include "SimplelHepticTrajectory.h"
#include <functional> 
#include <cmath>
#include <limits>


namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {
        SimplelHepticTrajectory::SimplelHepticTrajectory(double i_pos, double f_pos, double v_max, double a_max, double f_time)
            :initial_position(i_pos), target_position(f_pos), _final_time(f_time), max_velocity(v_max), max_acceleration( a_max)
        {
        }

        double SimplelHepticTrajectory::calculateMinTime(double vel_tolerance, double accel_tolerance)
        {
            if( (max_velocity <= abs( std::numeric_limits<double>::min()) )
            || (max_acceleration <= abs(std::numeric_limits<double>::min()) ))
            {
                return 0.00;
            }
            double delta_s = target_position - initial_position;             

            //Initial velocity, initial acceleration, final velocity and finL accelertations are all 0
            //maximums are in the middle -> we do not need solver

            double time_min = delta_s / max_velocity;
            double time_max = 2.00 * (max_velocity / max_acceleration) + time_min * 2.00;
            double guess = (time_max + time_min) / 2.00;
            create(guess);
            double time_optimal = 0.00;
                    
            double max_v = std::numeric_limits<double>::min();
            std::int32_t max_steps = 200;

           
            while (max_steps > 1)
            {           
                max_v = poly.firstDerivative(guess/2.0);
                double error = abs(max_velocity) - abs(max_v);

                if (abs(error) <= vel_tolerance)
                {
                    time_optimal = guess;
                    break;
                }

                if (error > 0.00)
                {
                    //slow
                    if (guess < time_max)
                    {
                        time_max =  guess;
                    }                  
                }
                else
                {
                    //to fast
                    if (guess > time_min)
                    {
                        time_min =  guess;
                    }
                }
                guess = (time_max + time_min) / 2.00;
                create(guess);
                max_steps--;
            }
            create(time_optimal);

            /*optimize for max_acceleration*/
            return time_optimal;
        }

        void SimplelHepticTrajectory::create(double f_time)
        {
            double a0 = 0.00;
            double a1 = 0.00;
            double a2 = 0.00;
            double a3 = 0.00;
            double a4 = 0.00;
            double a5 = 0.00;
            double a6 = 0.00;
            double a7 = 0.00;

            double tf = f_time;
            _final_time = f_time;
            double tf_2 = tf * tf;
            double tf_3 = tf_2 * tf;
            double tf_4 = tf_3 * tf;
            double tf_5 = tf_4 * tf;
            double tf_6 = tf_5 * tf;
            double tf_7 = tf_6 * tf;

            double delta_s = target_position - initial_position;
            a0 = initial_position;
            a1 = 0.00;;
            a2 = 0.00;
            a3 = 0.00;
            a4 = 35.0 * delta_s / (tf_4);
            a5 = -84.0 * delta_s / (tf_5);
            a6 = 70.0 * delta_s / (tf_6);;
            a7 = -20.0 * delta_s / tf_7;
            poly.setParams(a0, a1, a2, a3, a4, a5, a6, a7);
        }

        void SimplelHepticTrajectory::calculateValuesForTime(double t, double& position, double& velocity, double& acceleration, double& jerk)
        {
            double snap = 0.00;
            if (t > 0.00 && t <= _final_time)
            {
                poly.calculate(t, position, velocity, acceleration, jerk, snap);
            }            
        }
    }
}
