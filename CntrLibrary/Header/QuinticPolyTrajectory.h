#pragma once
#include "BasicNumMethods.h"
#include <cmath>
#include <algorithm>
#include "QuinticPolynomial.h"

using namespace CntrlLibrary::Math::BasicNumMethods;
using namespace CntrlLibrary::Math::Polynomials;


namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {
        /*very simple trajectory generation (Only velocity profile should be used in combination with additional position controller*/
        class QuinticPolyTrajectory
        {
        public:


            enum OptTimeResult
            {
                Converged = 1,
                NotConverged = 2,
                Error = 3
            };

            QuinticPolyTrajectory()
            {
            }


            /**
            * @brief Constructs a Quintic Polynomial Trajectory.
            *
            * Initializes a trajectory using a quintic polynomial based on the provided initial
            * and final conditions for position, velocity, and acceleration.
            *
            * @param i_pos    Initial position value for the trajectory.
            * @param i_vel    Initial velocity value for the trajectory.
            * @param i_accel  Initial acceleration value for the trajectory.
            * @param f_pos    Final position value for the trajectory.
            * @param f_vel    Final velocity value for the trajectory.
            * @param f_accel  Final acceleration value for the trajectory.
            * @param f_time   Time duration for which the trajectory is defined.
            */
            QuinticPolyTrajectory( double i_pos,
                                   double i_vel,
                                   double i_accel,
                                   double f_pos,
                                   double f_vel,
                                   double f_accel,
                                   double m_accel,
                                   double m_vel );

            void setParameters( double i_pos,
                                double i_vel,
                                double i_accel,
                                double f_pos,
                                double f_vel,
                                double f_accel,
                                double m_accel,
                                double m_vel);
            
            OptTimeResult calculateMinTime(double& minTime, double vel_tolerance, double accel_tolerance);
            
            /**
            * @brief Creates trajectory from final time
            **/
            void create(double f_time);

            /**
            * @brief Identify the extrema values for both velocity and acceleration.
            *
            * This function uses the Newton-Raphson method to find the extrema values of velocity and acceleration
            * based on a Quintic Polynomial. The initial values `initial_velocity` and `initial_acceleration` are
            * ignored during the calculations. Similarly, the final values `target_velocity` and
            * `target_acceleration` are not taken into account. A Quintic Polynomial can produce at most one
            * velocity extrema and two acceleration extremas.
            *
            * @param[out] ex_velocity          A pair representing the velocity extrema and its corresponding time.
            * @param[out] ex_accelerations     A pair of pairs representing the two acceleration extrema. 
            *                                  Each inner pair consists of an acceleration value and its corresponding time.
            *
            * @return ResultType from the Math::BasicNumMethods namespace, indicating the success or failure of
            *         the calculation.
            */
            Math::BasicNumMethods::ResultType findExtremaNewtonRaphson(std::pair<double, double>& ex_velocity, std::pair<std::pair<double, double>, std::pair<double, double>> & ex_accelerations);

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

            inline const QuinticPolynomial& getPoly() const
            {
                return poly;
            }

        private:

            double max_acceleration = 0.00;
            double max_velocity = 0.00;

            double initial_position = 0.00;
            double initial_velocity = 0.00;
            double initial_acceleration = 0.00;

            double target_position = 0.00;
            double target_velocity = 0.00;
            double target_acceleration = 0.00;

            double _final_time = 0.00;

            QuinticPolynomial poly;


        };
    }
}

