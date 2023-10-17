#ifdef TEST_VERSION

#include "JerkLimitedTrajectory.h"
#include <cmath>
#include <algorithm>

#define MIN_TIME_NUM_DIFF 0.00001
#define MIN_VAL_NUM_DIFF 0.00001



    namespace CntrlLibrary
    {

        /*This is only test and shuld not be used at the moment*/

        namespace TrajectoryGeneration
        {
            

            bool JerkLimitedTrajectory::prepare()
            {
               
                return false;
            }

         

            void JerkLimitedTrajectory::process(double t, double& position, double& velocity, double& acceleration)
            {
            }
        }
    }
#endif