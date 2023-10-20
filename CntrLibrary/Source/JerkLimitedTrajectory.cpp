#ifdef TEST_VERSION

#include "JerkLimitedTrajectory.h"
#include "QuinticPolyTrajectory.h"

#include <cmath>
#include <algorithm>

#define MIN_TIME_NUM_DIFF 0.00001
#define MIN_VAL_NUM_DIFF 0.00001

    namespace CntrlLibrary
    {

        /*This is only test and shuld not be used at the moment*/

        namespace TrajectoryGeneration
        {
        
            JerkLimitedTrajectory::JerkLimitedTrajectory()
            {

            }

            bool JerkLimitedTrajectory::prepare()
            {

                if (_tf <= std::numeric_limits<double>::min())
                {
                    return false;
                }

                if ((max_jerk <= std::numeric_limits<double>::min()) ||
                    max_acceleration <= std::numeric_limits<double>::min() ||
                    max_velocity <= std::numeric_limits<double>::min())
                {
                    return false;
                }
                                

                QuinticPolyTrajectory traj( initial_position,
                                            initial_velocity,
                                            initial_acceleration,
                                            target_position,
                                            target_velocity,
                                            target_acceleration,
                                            _tf);

                QuinticPolynomial poly(traj.getPoly());
                
                std::function<double(double)> funcCalcVel = std::bind(&QuinticPolynomial::calculateDerX, &poly, std::placeholders::_1);
                std::function<double(double)> funcCalcVelDer = std::bind(&QuinticPolynomial::calculateDerX2, &poly, std::placeholders::_1);
                        

                auto funcCalcVelAtMax = [funcCalcVel, this](double input)
                {
                    return (funcCalcVel(input) - max_velocity);
                };

                std::pair<double, double> ex_velocity;
                std::pair<std::pair<double, double>, std::pair<double, double>> ex_accelerations;
                Math::BasicNumMethods::ResultType result = traj.findExtremaNewtonRaphson(ex_velocity, ex_accelerations);
                double pos, vel, accel, jerk = 0.00;
              
                if (ex_velocity.first > max_velocity)
                {
                    //check if possible in given time
                    // 
                     //we need position (integrall of velocity)
                    traj.calculateValuesForTime(_tf, pos, vel, accel, jerk);
                    double v_eff = pos / _tf;
                    v_eff += 2.0 * v_eff* v_eff/ max_acceleration;
                    if (v_eff < max_velocity)
                    {
                        /*find the area where velocity > max_velocity*/
                        double T1 = ex_velocity.second - ex_velocity.second * (ex_velocity.first - max_velocity) / (max_velocity - initial_velocity);
                        double T2 = ex_velocity.second + (ex_velocity.first - max_velocity) * (_tf - ex_velocity.second) / (ex_velocity.first - target_velocity);

                        NewtonRaphson solverAccel(funcCalcVelAtMax, funcCalcVelDer, 1e-4, 100);
                        double root1 = 0.00;
                        double root2 = 0.00;
                        double initial_guess = T1;
                        result = solverAccel.findRoot(initial_guess, root1);
                        if (result != Math::BasicNumMethods::ResultType::Ok)
                        {
                            return false;
                        }
                        T1 = root1;

                        initial_guess = _tf - (_tf-T2)/2.00;
                        result = solverAccel.findRoot(initial_guess, root2);
                        if (result != Math::BasicNumMethods::ResultType::Ok)
                        {
                            return false;
                        }
                        T2 = root2;

                        traj.calculateValuesForTime(T1, pos, vel, accel, jerk);
                        double A1 = pos + (T2-T1) * max_velocity;

                        traj.calculateValuesForTime(_tf, pos, vel, accel, jerk);
                        double p = pos;
                        traj.calculateValuesForTime(T2, pos, vel, accel, jerk);
                        double pt2 = pos;
                        double A2 = p - pt2;
                        double bal = p - A1 - A2;
                        //....

                    }


                    // 
                   
                    traj.calculateValuesForTime(ex_velocity.second, pos, vel, accel,jerk );
                    v_eff = pos / ex_velocity.second;

                    int a = 0;
                    a++;
                }

                /*
               bool accelToHigh = false;
               bool hasNegAccel = false;
               bool hasPosAccel = false;
               for (double& am : maxAccels)
               {
                   if (am < (0.00 - MIN_VEL_NUMERIC))
                   {
                       hasNegAccel = true;
                   }
                   if (am > (0.00 + MIN_VEL_NUMERIC))
                   {
                       hasPosAccel = true;
                   }

                   if (std::abs(am) > max_acceleration)
                   {
                       accelToHigh = true;
                   }
               }*/

                return false;
            }

         

            void JerkLimitedTrajectory::process(double t, double& position, double& velocity, double& acceleration)
            {
            }
        }
    }
#endif