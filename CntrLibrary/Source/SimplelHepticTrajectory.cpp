#include "SimplelHepticTrajectory.h"
#include <functional> 
#include <cmath>
#include <limits>

#define TOLERANCE_COEFFICIENT 0.02 //0.01 i 1%  

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {
        SimplelHepticTrajectory::SimplelHepticTrajectory(double i_pos, double f_pos, double v_max, double a_max, double f_time)
            :initial_position(i_pos), target_position(f_pos), _final_time(f_time), max_velocity(v_max), max_acceleration( a_max)
        {
        }

        double SimplelHepticTrajectory::calculateMinTime()
        {
            if( (max_velocity <= abs( std::numeric_limits<double>::min()) )
            || (max_acceleration <= abs(std::numeric_limits<double>::min()) ))
            {
                return 0.00;
            }
            double delta_s = target_position - initial_position;             

            //Initial velocity, initial acceleration, final velocity and finL accelertations are all 0
            //maximums are in the middle -> we do not need solver
            

            std::function<double(double)> funcCalcVel = std::bind(&HepticPolynomial::calculateDerX, &poly, std::placeholders::_1);
                                  

            double time_min = delta_s / max_velocity;
            double time_max = 2.00 * (max_velocity / max_acceleration) + time_min * 2.00;
            create(time_min);
            double time_optimal = 0.00;

            double root2 = 0.00;
            double guess = time_min / 2.0;            
            double max_v = std::numeric_limits<double>::min();
            std::int32_t max_steps = 200;

            bool great = false;
            bool small = false;
            double last = 0.00;

            double prev_error = 0.00;
            double stepSize = time_max / 16.00;
            double tolerance = abs(max_velocity) * TOLERANCE_COEFFICIENT;
            while (max_steps > 1)
            {           
                max_v = poly.calculateDerX(guess);
                double error = abs(max_velocity) - abs(max_v);
                if (error > 0.00)
                {
                    if (abs(error) <= tolerance)
                    {
                        time_optimal = 2.0 * guess;
                        break;
                    }
                }

                // Adjust step size based on error trend
                if (error > 0.00 && error > prev_error)
                {
                    stepSize *= 0.5;  // Reduce step size by half
                }
                else if (error > 0.00 && error < prev_error)
                {
                    stepSize *= 1.1;  // Increase step size by 10%
                }
                else if (error < 0 && error < prev_error)
                {
                    stepSize *= 0.5;  // Reduce step size by half
                }
                else if (error < 0 && error > prev_error)
                {
                    stepSize *= 1.1;  // Increase step size by 10%
                }
                prev_error = error;

                // Clamp step size if necessary
                stepSize = std::max(0.001, std::min(stepSize, 0.50));

                // Modify time using the adaptive step size
                if (error > 0.00)
                {
                    guess = guess - stepSize;
                }
                else
                {
                    guess = guess + stepSize;
                }

                time_optimal = guess * 2.00;
                create(guess*2);
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
