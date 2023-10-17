#ifdef TEST_VERSION

#pragma once
#include <cmath>
#include <algorithm>

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {
        /*very simple trajectory generation (Only velocity profile should be used in combination with additional position controller*/
        class JerkLimitedTrajectory
        {
        public:

            JerkLimitedTrajectory();

            inline void setInitialConditions(double pos, double vel, double accel)
            {
                initial_position     = pos;
                initial_velocity     = vel;
                initial_acceleration = accel;
            }

            inline void setTargetPosition(double pos, double vel, double accel )
            {
                target_position     = pos;
                target_velocity     = vel;
                target_acceleration = accel;
            }

            inline void setParameters(double max_j, double max_a, double max_v, double ts)
            {
                _ts = ts;
                max_jerk            = max_j;
                max_acceleration    = max_a;
                max_velocity        = max_v;
            }

         
            /*function must be called once befor process*/
            bool prepare();
            
            void process(double t, double& position, double& velocity, double& acceleration);

           

        private:
            
           

        private:

            double _ts = 0.00001;

            double max_jerk         = 0.00;
            double max_acceleration = 0.00;
            double max_velocity     = 0.00;

            double initial_position = 0.00;
            double initial_velocity = 0.00;
            double initial_acceleration = 0.00;

            double target_position = 0.00;
            double target_velocity = 0.00;
            double target_acceleration = 0.00;


          };
    }
}
#endif