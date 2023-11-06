//#ifdef TEST_VERSION

#pragma once
#include <cmath>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include "PathSegment.h"

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {
        /*very simple trajectory generation (Only velocity profile should be used in combination with additional position controller*/
        class JerkLimitedTrajectory
        {
        public:

            enum ResultTrajectory
            {
                SafetyStop,
                SafetyVelocityReduction,
                TrangularTrajectory,
                TrapezoidTrajectory,
                AccelerationTrajectory,
                DeaccelerationTrajectory,
                NotPossible
            };

            JerkLimitedTrajectory();

            inline void setInitialValues(double pos, double vel, double accel)
            {
                _pos[START]     = pos;
                _vel[START]     = vel;
                _accel[START]   = accel;
            }

            inline void setFinalValues(double pos, double vel, double accel )
            {
                _pos[END]     = pos;
                _vel[END]     = vel;
                _accel[END]   = accel;
            }

            inline void setParameters(double max_j, double max_a, double max_v, double max_position_error)
            {               
                _max_jerk        = max_j;
                _max_accel       = max_a;
                _max_vel         = max_v;
                _max_pos_error   = max_position_error;
            }
                   

            /*function must be called once befor process*/
            /*if time is specified returns the same time if possible to create trajectory for time and otherconstrains. Otherwise returns new minimal possible time
            If time is not specified, returns minimal possible trajectory time
            Returns 0 if it is not possible to create trajectory with given initial and final values and constrains
            */                
            ResultTrajectory prepare( double& in_out_time);
            
            void process(double t, double& position, double& velocity, double& acceleration);

        private:


            struct FUNC_PARAMS
            {
                bool normalizedTime = false;
                bool inverseFunc = false;
                bool subtractInitialIntegral = false;
                double signA = 1.00;
                double signVel = 1.00;
                double signPos = 1.00;
                double scaleVel1 = 1.00;
                double scalePos1 = 1.00;
                double scalePos2 = 1.00;
                double startTime = 0.00;
                double endTime = 0.00;
                double dtime = 1.00;
                double dtime_squared = 1.00;
                double f_accel = 0.00;
                double f_vel = 0.00;
                double f_pos = 0.00;
            };

            void calculateJerkTimes(double Ac,
                double Dc,
                double& tphAp,
                double& tphAm,
                double& tphDp,
                double& tphDm);

            void calculateJerkDistances(double Ac,
                double Dc,
                double& tphAp,
                double& tphAm,
                double& tphDp,
                double& tphDm,
                double& aproxJerkDistAp,
                double& aproxJerkDistAm,
                double& aproxJerkDistDp,
                double& aproxJerkDistDm,
                double& aproxJerkDistance);

            bool findNewMaxVelocity(double travel_distance,
                double Ac,
                double Dc,
                double& new_max_velocity,
                double& aproxJerkDistAp,
                double& tphAp,
                double& tphAm,
                double& tphDp,
                double& tphDm,
                double& aproxJerkDistAm,
                double& aproxJerkDistDp,
                double& aproxJerkDistDm,
                double& aproxJerkDistance);

            bool findMaxAccelTimes(double travel_distance,
                double& Ac,
                double& Dc,
                double& tphAp,
                double& tphAm,
                double& tphDp,
                double& tphDm,
                bool& reduced);
            
            void addDMinusFuncParams();

            void addDPlusFuncParams();

            void addDConstFuncParams();

            double createStopTrajectory(double max_stop_distance);

            void clearMaps();

            void pathFunc(  double t, PathSegment* pathSegment, FUNC_PARAMS* params, double& a, double& v, double& s );

         private:

                        

            double _max_jerk         = 0.00;
            double _max_accel        = 0.00;
            double _max_vel          = 0.00;
            double _max_pos_error    = 0.000000001;
                       
            //Segments
            // 0 - start of initial position, vlocity and acceleration
            // 1 - end of +Acceleration
            // 2 - end of Const. Acceleration
            // 3 - end of -Acceleration
            // 4 - end of Const. Velocity
            // 5 - end of +Deacceleration
            // 6 - end of Const. Deacceleration
            // 7 - end of -Deacceleration (end position, end velocity, end acceleration )
            const int START = 0;
            const int APLUS = 1;
            const int AMINUS = 2;
            const int ACONST = 3;
            const int VCONST = 4;
            const int DPLUS = 5;
            const int DCONST = 6;
            const int DMINUS = 7;
            const int END = 8;

            //contains only the end times of each segment
            std::unordered_map<int, double>     _times;
            std::unordered_map<int, double>     _durations;

            std::unordered_map<int, double> _accel;
            std::unordered_map<int, double> _vel;
            std::unordered_map<int, double> _pos;
            
            std::unordered_map<int, std::unique_ptr<PathSegment>> _ptrSegments;

            std::unordered_map<int, std::shared_ptr<MathFunctionBase>> _functions;

            std::unordered_map<int, std::shared_ptr<FUNC_PARAMS>> _funcParams;

          };
    }
}
//#endif