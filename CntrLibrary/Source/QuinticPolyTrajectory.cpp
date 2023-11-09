#include "QuinticPolyTrajectory.h"
#include <functional> 
#include <cmath>
#include <limits>

#define TOLERANCE_COEFFICIENT 0.001 //0.01 i 0.1%  


#define MIN_VEL_NUMERIC 1e-5
#define MIN_ACCEL_NUMERIC 1e-5
#define MIN_TIME_DIFFERENCE 1e-5 /*10 us*/
#define MIN_PATH_PARTIAL_TIME 0.05e-3 /*500 us*/

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {

        QuinticPolyTrajectory::QuinticPolyTrajectory(   double i_pos,
                                                        double i_vel,
                                                        double i_accel,
                                                        double f_pos,
                                                        double f_vel,
                                                        double f_accel,
                                                        double m_accel,
                                                        double m_vel)
            : initial_position(i_pos)
            , initial_velocity(i_vel)
            , initial_acceleration(i_accel)
            , target_position(f_pos)
            , target_velocity(f_vel)
            , target_acceleration(f_accel)
            , max_acceleration(m_accel)
            , max_velocity(m_vel)
        {
        }

        void QuinticPolyTrajectory::setParameters(  double i_pos,
                                                    double i_vel,
                                                    double i_accel,
                                                    double f_pos,
                                                    double f_vel,
                                                    double f_accel,
                                                    double m_accel,
                                                    double m_vel)
        {
            initial_position = i_pos;
            initial_velocity = i_vel;
            initial_acceleration = i_accel;
            target_position = f_pos;
            target_velocity = f_vel;
            target_acceleration = f_accel;
            max_acceleration = m_accel;
            max_velocity = m_vel;
        }

        QuinticPolyTrajectory::OptTimeResult  QuinticPolyTrajectory::calculateMinTime(double& minTime, double vel_tolerance, double accel_tolerance)
        {
            if ((max_velocity <= abs(std::numeric_limits<double>::min()))
                || (max_acceleration <= abs(std::numeric_limits<double>::min())))
            {
                return OptTimeResult::Error;
            }
            double delta_s = target_position - initial_position;
            std::function<double(double)> funcCalcVel = std::bind(&QuinticPolynomial::firstDerivative, &poly, std::placeholders::_1);
            std::function<double(double)> funcCalcAccel = std::bind(&QuinticPolynomial::secondDerivative, &poly, std::placeholders::_1);
            std::function<double(double)> funcCalcJerk = std::bind(&QuinticPolynomial::thirdDerivative, &poly, std::placeholders::_1);
            std::function<double(double)> funcCalcJSnap = std::bind(&QuinticPolynomial::fourthDerivative, &poly, std::placeholders::_1);

            double time_min = minTime;
            double time_max = 2.00 * (max_velocity / max_acceleration) + time_min * 2.00;
            double guess = (time_min + time_max ) / 2.0;
            create(guess);

            std::pair<double, double> max_velo;
            std::pair<std::pair<double, double>, std::pair<double, double>> max_accels;
                        
            double time_optimal = 0.00;
            double max_v = std::numeric_limits<double>::min();
            std::int32_t max_steps = 200;
 
            bool great = false;
            bool small = false;
            double last = 0.00;

            double min_error = std::numeric_limits<double>::max();


            bool ok = false;
            while (max_steps > 1)
            {
                Math::BasicNumMethods::ResultType result = findExtremaNewtonRaphson(max_velo, max_accels);

                if (result != Math::BasicNumMethods::ResultType::Ok)
                {
                    return OptTimeResult::Error;
                }

                max_v = max_velo.first;

                double error = abs(max_velocity) - abs(max_v);
                if (abs(error) < abs(min_error))
                {
                    min_error = error;
                    minTime = guess;
                }

                if ( abs(error) <= vel_tolerance)
                {
                    time_optimal = 2.0 * max_velo.second;
                    minTime = time_optimal;
                    ok = true;
                    break;
                }

                if (error > 0.00)
                {
                    //slow
                    if (guess < time_max)
                    {
                        time_max = guess;
                    }
                }
                else
                {
                    //to fast
                    if (guess > time_min)
                    {
                        time_min = guess;
                    }
                }
                guess = (time_max + time_min) / 2.00;
                create(guess);
                max_steps--;
            }
            if (!ok)
            {
                time_optimal = minTime;
                //return OptTimeResult::NotConverged;
            }
            max_steps = 200;
            create(time_optimal);

            guess = time_optimal;
            double max_a = 0.00;
            /*limit max acceleration*/
            double time_max_accel = 0.00;
            double last_error = 0.00;
            double min_step = accel_tolerance / (2.0 * max_acceleration);
            double max_step = (2.0) * accel_tolerance / max_acceleration;
            double step = min_step;
            double min_time = time_optimal;
          
            while (max_steps > 1)
            {
                Math::BasicNumMethods::ResultType result = findExtremaNewtonRaphson(max_velo, max_accels);

                if (result != Math::BasicNumMethods::ResultType::Ok)
                {
                    return OptTimeResult::NotConverged;
                }
                if (abs(max_accels.first.first) > abs(max_accels.second.first))
                {
                    max_a = max_accels.first.first;
                    time_max_accel = max_accels.first.second;
                }
                else
                {
                    max_a = max_accels.second.first;
                    time_max_accel = max_accels.second.second;
                }
                double error = abs(max_acceleration) - abs(max_a);

                if (abs(error) <= abs(accel_tolerance))
                {
                    time_optimal = guess;
                    minTime = time_optimal;
                    return OptTimeResult::Converged;
                }

                // Adjust step if error is oscillating or not decreasing fast enough
                if (abs(error) > 0.9 * abs(last_error))
                {
                    step *= 0.50;  // reduce the step by half
                }
                // Ensure step is within bounds
                step = std::clamp(step, min_step, max_step);

                guess -= step * error;

                last_error = error;
           
                create(guess);
                max_steps--;
            }
            return OptTimeResult::NotConverged;
        }


        void QuinticPolyTrajectory::create(double f_time)
        {

            double tf = f_time;
            _final_time = f_time;

            double tf_2 = tf * tf;
            double tf_3 = tf_2 * tf;
            double tf_4 = tf_3 * tf;
            double tf_5 = tf_4 * tf;
          
            double a0 = initial_position;
            double a1 = initial_velocity;
            double a2 = initial_acceleration/2.00;
            double a3 = (20.0 * target_position - 20.0 * initial_position - (8.00 * target_velocity + 12.0 * initial_velocity) * tf - (3.00 * initial_acceleration - target_acceleration) * tf_2) / (2.00 * tf_3);
            double a4 = (30.0 * initial_position - 30.0 * target_position + (14.00 * target_velocity + 16.0 * initial_velocity) * tf + (3.00 * initial_acceleration - 2.0 * target_acceleration) * tf_2) / (2.00 * tf_4);;
            double a5 = (12.0 * target_position - 12.0 * initial_position - (6.00 * target_velocity + 6.0 * initial_velocity) * tf - (initial_acceleration - target_acceleration) * tf_2) / (2.00 * tf_5);;

            poly.setParams(a0, a1, a2, a3, a4, a5);

        }

        Math::BasicNumMethods::ResultType QuinticPolyTrajectory::findExtremaNewtonRaphson(std::pair<double, double>& ex_velocity, std::pair<std::pair<double, double>, std::pair<double, double>>& ex_accelerations)
        {
            Math::BasicNumMethods::ResultType result = Math::BasicNumMethods::ResultType::NotPossible;
            
            double a0 = poly.getA0();
            double a1 = poly.getA1();
            double a2 = poly.getA2();
            double a3 = poly.getA3();
            double a4 = poly.getA4();
            double a5 = poly.getA5();
            
            // Define the function of velocity
            std::function<double(double)> funcVel = [a0,a1,a2,a3,a4,a5](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x_2 * x;
                double x_4 = x_3 * x;
                return (a1 + 2.0 * a2 * x + 3.0 * a3 * x_2 + 4.00 * a4 * x_3 + 5 * a5 * x_4);
            };

            // Define the function of acceleration
            std::function<double(double)> funcAccel = [a2, a3, a4, a5](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x * x * x;
                return (2.0 * a2 + 6.0 * a3 * x + 12.00 * a4 * x_2 + 20.0 * a5 * x_3);
            };

            // Define the function of jerk
            std::function<double(double)> funcJerk = [a3, a4, a5](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x * x * x;
                return (6.0 * a3 + 24.00 * a4 * x + 60.0 * a5 * x * x);
            };

            // Define the function of snap
            std::function<double(double)> funcSnap = [a4, a5](double x) -> double
            {
                return (24.00 * a4 + 120.0 * a5 * x);
            };
            if (_final_time <= MIN_TIME_DIFFERENCE)
            {
                return Math::BasicNumMethods::ResultType::NotPossible;
            }
            double step = _final_time / 16.00;

            std::vector<double> rootsAccel;
            std::vector<double> rootsJerk;

            NewtonRaphson solverAccel(funcAccel, funcJerk, MIN_TIME_DIFFERENCE, 20);
            double root = 0.00;
            double initial_guess = 0.00;

            while (initial_guess <= (_final_time - step))
            {
                result = solverAccel.findRoot(initial_guess, root);
                if (result == Math::BasicNumMethods::ResultType::Ok)
                {
                    //we do not need the start of interval where t = 0 where the initial_velocity is defined 
                    if (abs(root) > MIN_TIME_DIFFERENCE)
                    {
                        //we do not need last point where the target_velocity is defined
                        if (root < (_final_time - MIN_TIME_DIFFERENCE))
                        {
                            rootsAccel.push_back(root);
                        }
                    }
                }                       
                initial_guess += step;
            }

            result = Math::BasicNumMethods::ResultType::NotPossible;
            NewtonRaphson solverJerk(funcJerk, funcSnap, MIN_TIME_DIFFERENCE, 20);
            root = 0.00;
            initial_guess = 0.00;
            while (initial_guess <= (_final_time - step))
            {
                result = solverJerk.findRoot(initial_guess, root);
                if (result == Math::BasicNumMethods::ResultType::Ok)
                {
                    if (false == (abs(root) <= std::numeric_limits<double>::min()))
                    {
                        rootsJerk.push_back(root);
                    }
                }               
                initial_guess += step;
            }
            std::sort(rootsAccel.begin(), rootsAccel.end());
            std::sort(rootsJerk.begin(), rootsJerk.end());

            std::function<bool(double, double, double)> isClose = [this](double a, double b, double min) -> bool
            {
                return std::abs(a - b) <= min;
            };

            //remove values that are close to each other based on a certain tolerance
            auto endAccel = std::unique(rootsAccel.begin(), rootsAccel.end(), [&](double a, double b) { return isClose(a, b, MIN_PATH_PARTIAL_TIME); });
            rootsAccel.erase(endAccel, rootsAccel.end());

            auto endJerk = std::unique(rootsJerk.begin(), rootsJerk.end(), [&](double a, double b) { return isClose(a, b, MIN_PATH_PARTIAL_TIME); });
            rootsJerk.erase(endJerk, rootsJerk.end());
                       
            uint32_t counter = 0U;
            for (auto& timea : rootsJerk)
            {
                if (counter == 0U)
                {
                    ex_accelerations.first.first = funcAccel(timea);
                    ex_accelerations.first.second = timea;
                }
                if (counter == 1U)
                {
                    ex_accelerations.second.first = funcAccel(timea);
                    ex_accelerations.second.second = timea;
                }
                if (counter > 1U)
                {
                    break;
                }
                counter++;
            }
            
            bool velToHigh = false;
            bool hasNegVel = false;
            bool hasPosVel = false;
            for (auto& timev : rootsAccel)
            {
                double maxVel = funcVel(timev);
                if (abs(maxVel) > MIN_VEL_NUMERIC)
                {
                    ex_velocity.first = maxVel;
                    ex_velocity.second = timev;
                    break;
                }
            }           
            return Math::BasicNumMethods::ResultType::Ok;
        }

        void QuinticPolyTrajectory::calculateValuesForTime(double t, double& position, double& velocity, double& acceleration, double& jerk)
        {
            if (t > 0.00 && t <= _final_time)
            {
                poly.calculate(t, position, velocity, acceleration, jerk);
            }
        }
    }
}










