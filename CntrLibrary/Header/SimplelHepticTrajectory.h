
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

          
            double calculateMinTime();

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
