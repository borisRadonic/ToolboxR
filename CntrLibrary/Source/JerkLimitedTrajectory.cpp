
//////////////////////////////////////////
//////WORK IN PROGRESS
// 
// Please do not use at the moment!
// ///////////////////////////////////////
///////////// NOR READY //////////////////
//////////////////////////////////////////




#include "JerkLimitedTrajectory.h"
#include "ConstFunction.h"
#include <cmath>
#include <algorithm>
#include <cassert>

namespace CntrlLibrary
{
    /*This is only test and shuld not be used at the moment*/

    using namespace Math;
    using namespace Bezier;
    using namespace TrajectoryGeneration;

    namespace TrajectoryGeneration
    {
        JerkLimitedTrajectory::JerkLimitedTrajectory()
        {
            _ptrSegments[APLUS] = std::make_unique<PathSegment>();
            _ptrSegments[ACONST] = std::make_unique<PathSegment>();
            _ptrSegments[AMINUS] = std::make_unique<PathSegment>();
            _ptrSegments[VCONST] = std::make_unique<PathSegment>();
            _ptrSegments[DPLUS] = std::make_unique<PathSegment>();
            _ptrSegments[DCONST] = std::make_unique<PathSegment>();
            _ptrSegments[DMINUS] = std::make_unique<PathSegment>();
        }

        void JerkLimitedTrajectory::calculateJerkTimes( double Ac,
                                                        double Dc,
                                                        double& tphAp,
                                                        double& tphAm,
                                                        double& tphDp,
                                                        double& tphDm)
        {
            /*minimum travel time will never be less then times required for jerk phases (even if all jerk phases do not exist)*/
            //The factor of 2.0 is caused by the Bézier curve.
            tphAp = 0.5 * abs(Ac - _accel[START]) / _max_jerk; //max. duration of + jerk phase
            tphAm = 0.5 * abs(Ac) / _max_jerk; //max. duration of - jerk phase
            tphDp = 0.5 * abs(Dc) / _max_jerk;  //max. duration of + jerk phase for deacceleration
            tphDm = 0.5 * (abs(Dc) - _accel[END]) / _max_jerk; //max. duration of - jerk phase for deacceleration

            
        }
        
        void JerkLimitedTrajectory::calculateAccelTimes(double Ac, double Dc, double tphAp, double tphDp, double& tphAc, double& tphDc, double max_v)
        {            
            tphAc = abs((max_v - _vel[START]) / Ac) - tphAp;
            tphDc = abs(abs(max_v - _vel[END]) / Dc) - tphDp;
        }

        void JerkLimitedTrajectory::calculateJerkDistances( double Ac,
                                                            double Dc,
                                                            double tphAp,
                                                            double tphAm,
                                                            double tphDp,                
                                                            double tphDm,                                                          
                                                            double& aproxJerkDistAp,
                                                            double& aproxJerkDistAm,
                                                            double& aproxJerkDistDp,
                                                            double& aproxJerkDistDm,
                                                            double& aproxJerkDistance,
                                                            double& velAp,
                                                            double& velAm,
                                                            double& velDp,
                                                            double& velDm )
        {
           //const acceleration and const deacceleration phases can not be calculated at this stage

            _utilBazierCurve.setParams(0.00, 0.00, 0.00, Ac, Ac, Ac );
            double sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphAp * tphAp;
            aproxJerkDistAp = tphAp * _vel[START] + sint + 0.50 * _accel[START] * tphAp * tphAp;
            velAp = _vel[START] + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphAp + _accel[START] * tphAp;

           
            velAm = velAp + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphAm;

            _utilBazierCurve.setParams(Ac, Ac, Ac, 0.00, 0.00, 0.00 );
            sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphAm * tphAm;
            aproxJerkDistAm = velAp * tphAm + sint;

            _utilBazierCurve.setParams(0.00, 0.00, 0.00, Dc, Dc, Dc);
            sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphDp * tphDp;
            aproxJerkDistDp = velAm * tphDp + sint;
            velDp = velAm + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphDp;

            _utilBazierCurve.setParams(0.00, 0.00, 0.00, Dc - _accel[END], Dc - _accel[END], Dc - _accel[END]);
            sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphDm * tphDm;
            aproxJerkDistDm = velDp * tphDm - sint;
            velDm = velDp + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphDm;

            aproxJerkDistance = aproxJerkDistAp + aproxJerkDistAm + aproxJerkDistDp + aproxJerkDistDm; 
        }


        void JerkLimitedTrajectory::calculateConstVelTime(  double Ac,
                                                            double Dc,
                                                            double Vc,
                                                            double tphAp,
                                                            double tphAc,
                                                            double tphAm,
                                                            double tphDp,
                                                            double tphDc,
                                                            double tphDm,
                                                            double& vc_time,
                                                            double& vc_dist,
                                                            double& velDp,
                                                            double& posAtEndAm,
                                                            double& distDm,
                                                            double& distDc,
                                                            double& distDp )
        {
            //calculate time for const velocity phase
            //correct jerk distances for velocities

            double distance_to_do = _pos[END] - _pos[START];

            double sign = 1.00;
            if (distance_to_do <= 0.00)
            {
                sign = -1.00;
            }

            _utilBazierCurve.setParams(0.00, 0.00, 0.00, Ac, Ac, Ac);
            double sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphAp * tphAp;
            double distAp  = tphAp * _vel[START] + sint + 0.50 * _accel[START] * tphAp * tphAp;
            double velAp = _vel[START] + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphAp + _accel[START] * tphAp;
            
            double distAc = 0.50 * Ac * tphAc * tphAc + velAp * tphAc;
            double velAc = velAp + Ac * tphAc;

            _utilBazierCurve.setParams(Ac, Ac, Ac, 0.00, 0.00, 0.00);
            sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphAm * tphAm;
  
            double distAm = velAc * tphAm + sint;
            double velAm = velAc + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphAm;

            _utilBazierCurve.setParams(0.00, 0.00, 0.00, Dc, Dc, Dc);
            sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphDp * tphDp;
            distDp =  velAm * tphDp + sint;
            velDp =  velAm + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphDp;

            distDc = velDp * tphDc + sign * 0.50 * Dc * tphDc * tphDc;
            //double velDc = velDp + Dc * tphAc;
            double velDc = velDp + Dc * tphDc;

            //just another angle of view (view from back)
            _utilBazierCurve.setParams(0.00, 0.00, 0.00, Dc - _accel[END], Dc - _accel[END], Dc - _accel[END]);
            sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphDm * tphDm;
            distDm = _vel[END] * tphDm - sint;
          
            double dis = abs(distAp) + abs(distAc) + abs(distAm) + abs(distDp) + abs(distDc) + abs(distDm);

            posAtEndAm = _pos[START] + sign * ( abs(distAp) + abs(distAc) + abs(distAm) );

            //calculate constant velocity distance
            vc_dist = abs(_pos[END] - _pos[START]) - abs(dis);
            vc_time = abs(vc_dist / Vc);

         }


        bool JerkLimitedTrajectory::findPeakVelocity(double travel_distance, double Ac, double Dc, double& p_velocity)
        {
            //find appropriate velocity
            const int MAX_STEPS = 1000;
            double min_v = 0.00;
            double max_v = _max_vel;
            bool reduced(false);
            uint32_t counter(0);

            double tphAp(0.00);
            double tphAm(0.00);
            double tphDp(0.00);
            double tphDm(0.00);
            double tphAc(0.00);
            double tphDc(0.00);

            double aproxJerkDistAp(0.00);
            double aproxJerkDistAm(0.00);
            double aproxJerkDistDp(0.00);
            double aproxJerkDistDm(0.00);
            double aproxJerkDistance(0.00);
            
            double velAp(0.00);
            double velAm(0.00);
            double velDp(0.00);
            double velDm(0.00);
            double velAc(0.00);
            double velDc(0.00);

            double distAp(0.00);
            double distAc(0.00);
            double distAm(0.00);
            double distDp(0.00);
            double distDc(0.00);
            double distDm(0.00);
            double dis(0.00);
            double sint(0.00);

            double sign = 1.00;
            if (travel_distance <= 0.00)
            {
                sign = -1.00;
            }

            calculateJerkTimes(Ac, Dc, tphAp, tphAm, tphDp, tphDm);

            //calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, aproxJerkDistAp, aproxJerkDistAm, aproxJerkDistDp, aproxJerkDistDm, aproxJerkDistance, velAp, velAm, velDp, velDm);
                     
            while (counter < MAX_STEPS)
            {
                counter++;
                
                calculateAccelTimes(Ac, Dc, tphAp, tphDp, tphAc, tphDc, p_velocity);

                _utilBazierCurve.setParams(0.00, 0.00, 0.00, Ac, Ac, Ac);
                sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphAp * tphAp;
                distAp = tphAp * _vel[START] + sint + 0.50 * _accel[START] * tphAp * tphAp;
                velAp = _vel[START] + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphAp + _accel[START] * tphAp;

                distAc = abs(0.50 * Ac * tphAc * tphAc) + velAp * tphAc;
                velAc = velAp + Ac * tphAc;

                distAm = velAc * tphAm + sint;
                velAm = velAc + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphAm;

                _utilBazierCurve.setParams(0.00, 0.00, 0.00, Dc, Dc, Dc);
                sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphDp * tphDp;
                distDp = velAm * tphDp + sint;
                velDp = velAm + (_utilBazierCurve.firstIntegral(1.00) - _utilBazierCurve.firstIntegral(0.00)) * tphDp;

                distDc = velDp * tphDc - abs(0.50 * Dc * tphDc * tphDc);
                velDc = velDp + Dc * tphAc;

                //just another angle of view (view from back)
                _utilBazierCurve.setParams(0.00, 0.00, 0.00, Dc - _accel[END], Dc - _accel[END], Dc - _accel[END]);
                sint = (_utilBazierCurve.secondIntegral(1.00) - _utilBazierCurve.secondIntegral(0.00)) * tphDm * tphDm;
                distDm = _vel[END] * tphDm - sint;

                dis = distAp + distAc + distAm + distDp + distDc + distDm;


                //calculate constant velocity distance
                double vc_dist = abs(_pos[END] - _pos[START]) - abs(dis);

                double difference = abs(travel_distance) - dis;
                if ( abs(difference) < _max_pos_error)
                {
                    p_velocity = sign * p_velocity;
                    return true;
                }
                if (difference <= 0.00)
                {
                    reduced = true;
                    max_v = p_velocity;
                    p_velocity = (min_v + max_v) / 2.00;
                }
                else
                {
                    if (p_velocity >= (_max_vel - _max_vel_error))
                    {
                        p_velocity = sign * p_velocity;
                        return true;
                    }
                    if (reduced)
                    {
                        min_v = p_velocity;
                        p_velocity = (min_v + max_v) / 2.00;
                    }
                }
            }
            p_velocity = sign * p_velocity;
            return false;
        }

        bool JerkLimitedTrajectory::findMaxAccelTimes(double travel_distance,
            double& Ac,
            double& Dc,
            double& tphAp,
            double& tphAm,
            double& tphDp,
            double& tphDm,
            bool& reduced)
        {
            const int MAX_STEPS = 1000;
            double sign = 1.00;
            if (travel_distance <= 0.00)
            {
                sign = -1.00;
            }
            Ac = sign * _max_accel;
            Dc = -sign * _max_accel;

            double min_dc = 0.00;
            double max_dc = Dc;
            double max_ac = Ac;
            double min_ac = 0.00;

            double velAp(0.00);
            double velAm(0.00);
            double velDp(0.00);
            double velDm(0.00);

            reduced = false;
            uint32_t counter(0);

            double aproxJerkDistAp(0.00);
            double aproxJerkDistAm(0.00);
            double aproxJerkDistDp(0.00);
            double aproxJerkDistDm(0.00);
            double aproxJerkDistance(0.00);

            double deltaV = _vel[END] - _vel[START];
            double deltaV_squ = deltaV * deltaV;
            double distDv(0.00);

            while (counter < MAX_STEPS)
            {
                counter++;
                calculateJerkTimes(Ac, Dc, tphAp, tphAm, tphDp, tphDm);
                calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, aproxJerkDistAp, aproxJerkDistAm, aproxJerkDistDp, aproxJerkDistDm, aproxJerkDistance, velAp, velAm, velDp, velDm);

                //calculate the impact of velocity change
                if (deltaV < 0.00)
                {
                    distDv = deltaV_squ / (2.0 * abs(Dc));
                }
                else
                {
                    distDv = deltaV_squ / (2.0 * abs(Ac));
                }
                aproxJerkDistance = aproxJerkDistAp + aproxJerkDistAm + aproxJerkDistDp + aproxJerkDistDm - distDv;
                double difference = travel_distance - abs(aproxJerkDistance);
                if (difference <= 0.00)
                {
                    reduced = true;
                    max_dc = Dc;
                    max_ac = Ac;
                    Dc = (min_dc + max_dc) / 2.00;
                    Ac = (min_ac + max_ac) / 2.00;
                }
                else
                {
                    if (abs(difference) <= abs(_max_pos_error))
                    {
                        return true;
                    }
                    if (reduced)
                    {
                        min_dc = Dc;
                        min_ac = Ac;
                        Dc = (min_dc + max_dc) / 2.00;
                        Ac = (min_ac + max_ac) / 2.00;
                    }
                }
            }
            return false;
        }

        void JerkLimitedTrajectory::addAPlusFuncParams(double startTime)
        {
            double tphAp = _times[APLUS] - startTime;
            assert(tphAp > 0.00);
            _funcParams[APLUS] = std::make_shared<FUNC_PARAMS>();
            _funcParams[APLUS]->normalizedTime = true;
            _funcParams[APLUS]->inverseFunc = false;
            _funcParams[APLUS]->subtractInitialIntegral = true;
            _funcParams[APLUS]->signA = 1.00;
            _funcParams[APLUS]->signVel = 1.00;
            _funcParams[APLUS]->signPos = 1.00;
            _funcParams[APLUS]->scaleVel1 = 1.00;
            _funcParams[APLUS]->scalePos1 = 1.00;
            _funcParams[APLUS]->scalePos2 = 1.00;
            _funcParams[APLUS]->startTime = startTime;
            _funcParams[APLUS]->endTime = _times[APLUS];
            _funcParams[APLUS]->dtime = tphAp;
            _funcParams[APLUS]->dtime_squared = tphAp * tphAp;
            _funcParams[APLUS]->f_accel = 0.00;
            _funcParams[APLUS]->f_vel = 0.00;
            _funcParams[APLUS]->f_pos = 0.00;
        }

        void JerkLimitedTrajectory::addAMinusFuncParams(double startTime)
        {
            double tphAm = _times[AMINUS]- startTime;
            assert(tphAm > 0.00);
            _funcParams[AMINUS] = std::make_shared<FUNC_PARAMS>();
            _funcParams[AMINUS]->normalizedTime = true;
            _funcParams[AMINUS]->inverseFunc = false;
            _funcParams[AMINUS]->subtractInitialIntegral = true;
            _funcParams[AMINUS]->signA = 1.00;
            _funcParams[AMINUS]->signVel = 1.00;
            _funcParams[AMINUS]->signPos = 1.00;
            _funcParams[AMINUS]->scaleVel1 = 1.00;
            _funcParams[AMINUS]->scalePos1 = 1.00;
            _funcParams[AMINUS]->scalePos2 = 1.00;
            _funcParams[AMINUS]->startTime = startTime;
            _funcParams[AMINUS]->endTime = _times[AMINUS];
            _funcParams[AMINUS]->dtime = tphAm;
            _funcParams[AMINUS]->dtime_squared = tphAm * tphAm;
            _funcParams[AMINUS]->f_accel = 0.00;
            _funcParams[AMINUS]->f_vel = 0.00;
            _funcParams[AMINUS]->f_pos = 0.00;
        }


        void JerkLimitedTrajectory::addDMinusFuncParams(double startTime)
        {
            double tphDm = _times[DMINUS] - startTime;
            assert(tphDm > 0.00);
            _funcParams[DMINUS] = std::make_shared<FUNC_PARAMS>();
            _funcParams[DMINUS]->normalizedTime = true;
            _funcParams[DMINUS]->inverseFunc = false;
            _funcParams[DMINUS]->subtractInitialIntegral = true;
            _funcParams[DMINUS]->signA = 1.00;
            _funcParams[DMINUS]->signVel = 1.00;
            _funcParams[DMINUS]->signPos = 1.00;
            _funcParams[DMINUS]->scaleVel1 = 1.00;
            _funcParams[DMINUS]->scalePos1 = 1.00;
            _funcParams[DMINUS]->scalePos2 = 1.00;
            _funcParams[DMINUS]->startTime = startTime;
            _funcParams[DMINUS]->endTime = _times[DMINUS];
            _funcParams[DMINUS]->dtime = tphDm;
            _funcParams[DMINUS]->dtime_squared = tphDm * tphDm;
            _funcParams[DMINUS]->f_accel = 0.00/*_accel[DCONST]*/;
            _funcParams[DMINUS]->f_vel = 0.00/*_vel[END];*/;
            _funcParams[DMINUS]->f_pos = 0.00/*_pos[END];*/;
        }


        void JerkLimitedTrajectory::addDMinusFuncParamsStop(double startTime)
        {
            double tphDm = _times[DMINUS] - startTime;
            assert(tphDm > 0.00);
            _funcParams[DMINUS] = std::make_shared<FUNC_PARAMS>();
            _funcParams[DMINUS]->normalizedTime = true;
            _funcParams[DMINUS]->inverseFunc = true;
            _funcParams[DMINUS]->subtractInitialIntegral = true;
            _funcParams[DMINUS]->signA = 1.00;
            _funcParams[DMINUS]->signVel = 1.00;
            _funcParams[DMINUS]->signPos = 1.00;
            _funcParams[DMINUS]->scaleVel1 = 1.00;
            _funcParams[DMINUS]->scalePos1 = 1.00;
            _funcParams[DMINUS]->scalePos2 = 1.00;
            _funcParams[DMINUS]->startTime = startTime;
            _funcParams[DMINUS]->endTime = _times[DMINUS];
            _funcParams[DMINUS]->dtime = tphDm;
            _funcParams[DMINUS]->dtime_squared = tphDm * tphDm;
            _funcParams[DMINUS]->f_accel = _accel[DCONST];
            _funcParams[DMINUS]->f_vel = _vel[END];
            _funcParams[DMINUS]->f_pos = _pos[END];
        }


        void JerkLimitedTrajectory::addDPlusFuncParams(double startTime)
        {
            double tphDp = _times[DPLUS] - startTime;
            assert(tphDp > 0.00);
            _funcParams[DPLUS] = std::make_shared<FUNC_PARAMS>();
            _funcParams[DPLUS]->normalizedTime = true;
            _funcParams[DPLUS]->inverseFunc = false;
            _funcParams[DPLUS]->subtractInitialIntegral = true;
            _funcParams[DPLUS]->signA = 1.00;
            _funcParams[DPLUS]->signVel = 1.00;
            _funcParams[DPLUS]->signPos = 1.00;
            _funcParams[DPLUS]->scaleVel1 = 1.00;
            _funcParams[DPLUS]->scalePos1 = 1.00;
            _funcParams[DPLUS]->scalePos2 = 1.00;
            _funcParams[DPLUS]->startTime = startTime;
            _funcParams[DPLUS]->endTime = _times[DPLUS];
            _funcParams[DPLUS]->dtime = tphDp;
            _funcParams[DPLUS]->dtime_squared = tphDp * tphDp;
            _funcParams[DPLUS]->f_accel = 0.00;
            _funcParams[DPLUS]->f_vel = 0.00;
            _funcParams[DPLUS]->f_pos = 0.00;
        }

        void JerkLimitedTrajectory::addDConstFuncParams(double startTime)
        {
            double tphDc = _times[DCONST] - startTime;
            assert(tphDc > 0.00);
            _funcParams[DCONST] = std::make_shared<FUNC_PARAMS>();
            _funcParams[DCONST]->normalizedTime = false;
            _funcParams[DCONST]->inverseFunc = false;
            _funcParams[DCONST]->subtractInitialIntegral = false;
            _funcParams[DCONST]->signA = 1.00;
            _funcParams[DCONST]->signVel = 1.00;
            _funcParams[DCONST]->signPos = 1.00;
            _funcParams[DCONST]->scaleVel1 = 1.00;
            _funcParams[DCONST]->scalePos1 = 1.00;
            _funcParams[DCONST]->scalePos2 = 1.00;
            _funcParams[DCONST]->startTime = startTime;
            _funcParams[DCONST]->endTime = _times[DCONST];
            _funcParams[DCONST]->dtime = tphDc;
            _funcParams[DCONST]->dtime_squared = tphDc * tphDc;
            _funcParams[DCONST]->f_accel = 0.00;
            _funcParams[DCONST]->f_vel = 0.00;
            _funcParams[DCONST]->f_pos = 0.00;
        }

        void JerkLimitedTrajectory::addAConstFuncParams(double startTime)
        {
            double tphDc = _times[ACONST] - startTime;
            assert(tphDc > 0.00);
            _funcParams[ACONST] = std::make_shared<FUNC_PARAMS>();
            _funcParams[ACONST]->normalizedTime = false;
            _funcParams[ACONST]->inverseFunc = false;
            _funcParams[ACONST]->subtractInitialIntegral = false;
            _funcParams[ACONST]->signA = 1.00;
            _funcParams[ACONST]->signVel = 1.00;
            _funcParams[ACONST]->signPos = 1.00;
            _funcParams[ACONST]->scaleVel1 = 1.00;
            _funcParams[ACONST]->scalePos1 = 1.00;
            _funcParams[ACONST]->scalePos2 = 1.00;
            _funcParams[ACONST]->startTime = startTime;
            _funcParams[ACONST]->endTime = _times[ACONST];
            _funcParams[ACONST]->dtime = tphDc;
            _funcParams[ACONST]->dtime_squared = tphDc * tphDc;
            _funcParams[ACONST]->f_accel = 0.00;
            _funcParams[ACONST]->f_vel = 0.00;
            _funcParams[ACONST]->f_pos = 0.00;
        }

        void JerkLimitedTrajectory::addVConstFuncParams(double startTime)
        {
            double tphVc = _times[VCONST] - startTime;
            
            _funcParams[VCONST] = std::make_shared<FUNC_PARAMS>();
            _funcParams[VCONST]->normalizedTime = false;
            _funcParams[VCONST]->inverseFunc = false;
            _funcParams[VCONST]->subtractInitialIntegral = false;
            _funcParams[VCONST]->signA = 1.00;
            _funcParams[VCONST]->signVel = 1.00;
            _funcParams[VCONST]->signPos = 1.00;
            _funcParams[VCONST]->scaleVel1 = 1.00;
            _funcParams[VCONST]->scalePos1 = 1.00;
            _funcParams[VCONST]->scalePos2 = 1.00;
            _funcParams[VCONST]->startTime = startTime;
            _funcParams[VCONST]->endTime = _times[VCONST];
            _funcParams[VCONST]->dtime = tphVc;
            _funcParams[VCONST]->dtime_squared = tphVc * tphVc;
            _funcParams[VCONST]->f_accel = 0.00;
            _funcParams[VCONST]->f_vel = 0.00;
            _funcParams[VCONST]->f_pos = 0.00;
        }

           double JerkLimitedTrajectory::createTrajectory(double Ac, double Dc, double Vc)
        {            
            double distAp(0.00);
            double distAm(0.00);
            double distDp(0.00);
            double distDm(0.00);
            double velAp(0.00);
            double velAm(0.00);
            double velDp(0.00);
            double velDm(0.00);
            double distance(0.00);
            double tphAp(0.00);
            double tphAm(0.00);
            double tphDp(0.00);
            double tphDm(0.00);

            double posAtEndAm(0.00);

            /*
            * 
            * Velocity
            *
            *                ********
            * ^            *           *
            *            *               *
            * |        *                   *
            * |       *                      *
            * |     *                          *
            * |   *                              *
            * |*A+   Ac    A-*    Vc   D+    Dc  D-*
            * +--------------------------------------->
           
            /*
            * Acceleration
            * ^
            * |      ***
            * |    *     *
            * |   *       *
            * |  *         *
            * |*A+   Ac    A-*    Vc   D+    Dc   D-
            * +--------------------------------------->
            * |                        *            *
            * |                          *        *
            * |                           *      *
            * |                            *    *
            * |                             ***

            /*
            * Jerk
            * ^
            * |  **                    **
            * | *  *                  *  *
            * | *  *                  *  *
            * | *  *                  *  *
            * |* A+ *                * D+ *
            * +------------------------------------->
            * |          * A- *               * D- *
            * |           *  *                 *  *
            * |           *  *                 *  *
            * |           *  *                 *  *
            * |            **                   **
            */
            
            calculateJerkTimes(Ac, Dc, tphAp, tphAm, tphDp, tphDm);

            double distance_to_do = _pos[END] - _pos[START];

            double sign = 1.00;
            if (distance_to_do <= 0.00)
            {
                sign = -1.00;
            }

            
            calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, distAp, distAm, distDp, distDm, distance, velAp, velAm, velDp, velDm);

            double delta_v_Ap = velAp;
            double delta_v_Am = velAm - velAp;

            //acceleration phase A+
            if (tphAp > std::numeric_limits<double>::min())
            {
                _times[APLUS] = tphAp;
                std::shared_ptr<QuinticBezierCurve> fpap = std::make_shared<QuinticBezierCurve>();
                fpap->setParams(0.00, 0.00, 0.00, Ac, Ac, Ac);
                _functions[APLUS] = fpap;
                _ptrSegments[APLUS]->create(0.00, 1.00, _accel[START], _vel[START], _pos[START], _functions[APLUS]);
                addAPlusFuncParams(0.00);
                _vel[APLUS] = velAp;
                _accel[APLUS] = Ac;
                _pos[APLUS] = _pos[START] + distAp;
            }
            else
            {
                //funny case (start acceleration is alredy max.)
            }
            double tphAc(0.00);
            double tphDc(0.00);

            calculateAccelTimes(Ac, Dc, tphAp, tphDp, tphAc, tphDc, Vc);

            //const velocity phase
            double tphVc(0.00);
            double vc_dist(0.00);
            double distDc(0.00);

            calculateConstVelTime(Ac, Dc, Vc, tphAp, tphAc, tphAm, tphDp, tphDc, tphDm, tphVc, vc_dist, velDp, posAtEndAm, distDm, distDc, distDp);
            
            //const acceleration phase
            assert(tphAc > std::numeric_limits<double>::min());
            _functions[ACONST] = std::make_shared<ConstFunction>(Ac);
            _accel[ACONST] = Ac;
            _times[ACONST] = _times[APLUS] + tphAc;
            _ptrSegments[ACONST]->create(_times[APLUS], _times[ACONST], 0.00, _vel[APLUS], _pos[APLUS], _functions[ACONST]);
            addAConstFuncParams(_times[APLUS]);
            _vel[ACONST] = velAp + Ac * tphAc;
            _pos[ACONST] = _pos[APLUS] + 0.50 * Ac * tphAc * tphAc + velAp * tphAc;


            //acceleration phase A-
            assert(tphAm > std::numeric_limits<double>::min());
            std::shared_ptr<QuinticBezierCurve> fpam = std::make_shared<QuinticBezierCurve>();
            fpam->setParams(Ac, Ac, Ac, 0.00, 0.00, 0.00 );
            _functions[AMINUS] = fpam;
            _ptrSegments[AMINUS]->create(0.00, 1.00, 0.00, _vel[ACONST], _pos[ACONST], _functions[AMINUS]);
            _times[AMINUS] = _times[ACONST] + tphAm;
            addAMinusFuncParams(_times[ACONST]);
            _vel[AMINUS] = _vel[ACONST] + delta_v_Am;
                       
            _accel[AMINUS] = 0.00;


            //double distAm = velAc * tphAm + sint;
            //double distAp = tphAp * _vel[START] + sint + 0.50 * _accel[START] * tphAp * tphAp;
            //double distAc = 0.50 * Ac * tphAc * tphAc + velAp * tphAc;
            //posAtEndAm = _pos[ACONST] + abs(distAm) + _vel[AMINUS] * tphAm;
            _pos[AMINUS] = posAtEndAm;
            //_pos[AMINUS] = _pos[ACONST] + 

            assert( abs( abs(_vel[AMINUS]) - abs(Vc))  <= _max_vel_error);

            /*
            If we can not reach max velocity :


            We determine Feasible Peak Velocity calculating the highest velocity that can be achieved given the constraints.

                Adjust Accelerationand Deceleration Phases  so that the system smoothly accelerates to this peak velocityand then decelerates to the final velocity.

                Eliminate Constant Velocity Phase!If the maximum velocity cannot be reached, there will be no constant velocity phase.

                The durations of the accelerationand deceleration segments need to be recalculated based on the new velocity profile.
                Check that the recalculated trajectory meets the desired end conditions for positionand velocity.If not, further adjustments may be necessary.

                Checking is there a need for a constant velocity phase :

            1. Calculate the Distance Required for Acceleration determining the distance needed to accelerate from the initial velocity to the maximum velocity under the maximum allowable accelerationand jerk.

                2. Calculate the Distance Required for Deceleration calculating the distance required to decelerate from the maximum velocity to the final velocity under the maximum allowable decelerationand jerk.

                3. Add the distances calculated for accelerationand deceleration.If the sum is less than the total distance to be covered, there is room for a constant velocity phase

                4. If the sum of the acceleration and deceleration distances is less than the total distance, but the maximum velocity is too high to be sustained(due to other constraints like system limitations or a speed limit),
                we may need to calculate a lower 'feasible' maximum velocity.
                */


                       ;
          
            _functions[VCONST] = std::make_shared<ConstFunction>(0.00);
            _times[VCONST] = _times[AMINUS] + tphVc;
            _ptrSegments[VCONST]->create(_times[AMINUS], _times[VCONST], 0.00, Vc, _pos[AMINUS], _functions[VCONST]);
            addVConstFuncParams(_times[AMINUS]);
            _vel[VCONST] = Vc;
            _pos[VCONST] = _pos[AMINUS] + Vc * tphVc;
            _accel[VCONST] = 0.00;

            assert(tphDp > std::numeric_limits<double>::min());

            //deceleration D+ phase
            std::shared_ptr<QuinticBezierCurve> fpdp = std::make_shared<QuinticBezierCurve>();
            fpdp->setParams(0.00, 0.00, 0.00, Dc, Dc, Dc);
            _functions[DPLUS] = fpdp;
            _ptrSegments[DPLUS]->create(0.00, 1.00, 0.00, _vel[VCONST], _pos[VCONST], _functions[DPLUS]);
            _times[DPLUS] = _times[VCONST] + tphDp;
            addDPlusFuncParams(_times[VCONST]);
            _vel[DPLUS] = velDp;
            _pos[DPLUS] = _pos[VCONST] + distDp;
            _accel[DPLUS] = Dc;

            //constant deceleration  phase
            assert(tphDc > std::numeric_limits<double>::min());
            _functions[DCONST] = std::make_shared<ConstFunction>(Dc);
            _accel[DCONST] = Dc;
            _times[DCONST] = _times[DPLUS] + tphDc;
            _ptrSegments[DCONST]->create(_times[DPLUS], _times[DCONST], 0.00, _vel[DPLUS], _pos[DPLUS], _functions[DCONST]);
            addDConstFuncParams(_times[DPLUS]);

            _vel[DCONST] = _ptrSegments[DCONST]->getVelocity(_times[DCONST], 1.00, false) ;
            _pos[DCONST] = _pos[DPLUS] + 0.50 * Dc * tphDc * tphDc + velDp * tphDc;

    
            std::shared_ptr<QuinticBezierCurve> fpdm = std::make_shared<QuinticBezierCurve>();
             fpdm->setParams( Dc + _accel[END], Dc + _accel[END], Dc + _accel[END],0.00, 0.00, 0.00);
            _functions[DMINUS] = fpdm;
           
          
            _ptrSegments[DMINUS]->create(0.00, 1.00, 0.00, _vel[DCONST], _pos[DCONST], _functions[DMINUS]);
            _times[DMINUS] = _times[DCONST] + tphDm;
            addDMinusFuncParams(_times[DCONST]);
            _pos[DMINUS] = _pos[DCONST] + distDm;
            _accel[DMINUS] = _accel[END];
        
            double endTime = tphAp + tphAm + tphAc + tphDp + tphDc + tphDm + tphVc;
                     
            return (endTime);
        }

       

        double JerkLimitedTrajectory::createStopTrajectory(double max_stop_distance, 
                                                           double Dc,
                                                           double tphDp,
                                                           double tphDm,
                                                           double tphDc,
                                                           double velocityDp,
                                                           double velocityDm,
                                                            double posDp,
                                                           std::shared_ptr<QuinticBezierCurve> fpdm,
                                                           std::shared_ptr<QuinticBezierCurve> fpdp)
        {
            //create stop trajectory
                       
            _vel[END] = 0.00;
            _accel[END] = 0.00;

            if (_vel[START] >= 0.00)
            {
                _pos[END] = _pos[START] + max_stop_distance;
            }
            else
            {
                _pos[END] = _pos[START] - max_stop_distance;
            }

            double sign(1.00);
            if (_vel[START] < 0.00)
            {
                sign = -1.00;
            }
             Dc = -sign * abs(Dc);
                      
            //Phases D+, Dconst, D-

            //time at the end od D+
            _functions[DPLUS] = fpdp;
            _times[DPLUS] = tphDp;
            _vel[DPLUS] = _vel[START] + velocityDp;
            _pos[DPLUS] = posDp;

            _functions[DMINUS] = fpdm;
            _vel[DMINUS] = velocityDm;

            _ptrSegments[DPLUS]->create(0.00, 1.00, _accel[START], _vel[START], _pos[START], _functions[DPLUS]);

            //time at the end of D const
            _times[DCONST] = _times[DPLUS] + tphDc;

           
            //time at the end of D-  (End Time)
            _times[DMINUS] = _times[DCONST] + tphDm;
            
            _accel[DPLUS] = Dc;
            _accel[DCONST] = Dc;
            _functions[DCONST] = std::make_shared<ConstFunction>(Dc);
            _ptrSegments[DCONST]->create(_times[DPLUS], _times[DCONST], 0.00, _vel[DPLUS], _pos[DPLUS], _functions[DCONST]);
            
            _vel[DCONST] = _vel[START] - sign * abs(_vel[DPLUS]);
            _ptrSegments[DMINUS]->create(0.00, 1.00, _accel[DCONST], 0.00, 0.00, _functions[DMINUS]);

            addDPlusFuncParams(0.00);
            addDConstFuncParams(_times[DPLUS]);
            addDMinusFuncParamsStop(_times[DCONST]);

            return (_times[DMINUS]);
        }

        void JerkLimitedTrajectory::clearMaps()
        {
            //first reset all function pointers ( Object could be already used)
            for (auto& pair_ifunc : _functions)
            {
                pair_ifunc.second.reset();
            }

            for (auto& pair_ifunc_params : _funcParams)
            {
                pair_ifunc_params.second.reset();
            }
            
            _funcParams.clear();
            _times.clear();

            /*clear all values, but not START and END */
            for (auto it = _accel.begin(); it != _accel.end(); )
            {
                if (it->first != START && it->first != END)
                {
                    it = _accel.erase(it);
                }
                else
                {
                    ++it;
                }
            }

            for (auto it = _vel.begin(); it != _vel.end(); )
            {
                if (it->first != START && it->first != END)
                {
                    it = _vel.erase(it);
                }
                else
                {
                    ++it;
                }
            }

            for (auto it = _pos.begin(); it != _pos.end(); )
            {
                if (it->first != START && it->first != END)
                {
                    it = _pos.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        JerkLimitedTrajectory::ResultTrajectory JerkLimitedTrajectory::prepare(double& in_out_time)
        {            
            //clean up old values

            _use_quinticPoly = false;

            clearMaps();

            double travel_distance = _pos[END] - _pos[START];

            if (abs(travel_distance) <= std::numeric_limits<double>::min())
            {
                return ResultTrajectory::NotPossible;
            }

            //check input parameters

            /// Check for exceeding maximum acceleration
            
            if ( (std::abs(_accel[END]) > _max_accel) )
            {
                return ResultTrajectory::NotPossible;
            }

            /// Check for exceeding maximum acceleration
            // This is a simplified check assuming constant acceleration
            if (std::abs(_accel[END]) > _max_accel)
            {
                return ResultTrajectory::NotPossible;
            }

            // Check for exceeding maximum jerk
            // This is a simplified check assuming constant jerk
            if (in_out_time > std::numeric_limits<double>::min())
            {
                double delta_a = _accel[END] - _accel[START];
                double required_jerk = delta_a / in_out_time;
                if (std::abs(required_jerk) > _max_jerk)
                {
                    return ResultTrajectory::NotPossible;
                }
            }
                        
            /*
            When we can not reach max acceleration:
            
                Recalculate Maximum Feasible Acceleration:
                Determine the highest acceleration that can be achieved without violating the jerk constraint over the distance to be covered. 
                This can be done by considering the initial and final velocities, the distance to be covered, and the maximum jerk.
            
                If the maximum acceleration cannot be reached, there may not be a constant acceleration phase.


                With the new acceleration profiles, recalculate the velocity profiles.
                The maximum velocity achieved may be less than the maximum allowable velocity due to the lower acceleration.
            
                The durations of each segment need to be recalculated based on the new acceleration and velocity profiles. Ensure that the sum of the distances covered
                in all segments equals the total distance.
           */
            
            double Ac = 0.00;
            double Dc = 0.00;
            double Vc = 0.00;

            double sign = 1.00;
            if (travel_distance <= 0.00)
            {
                sign = -1.00;
            }
            Ac = sign * _max_accel;
            Dc = -sign * _max_accel;

            double start_velocity_squ = _vel[START] * _vel[START];

            double tphAp(0.00);
            double tphAm(0.00);
            double tphDp(0.00);
            double tphDm(0.00);


            if (((_vel[START] * _vel[START]) / (2.0 * _max_accel)) > abs(travel_distance))
            {

                /*check if safety stop required*/
                tphDp = abs(Dc - _accel[START]) / _max_jerk;
                tphDm = abs(Dc) / _max_jerk;

                std::shared_ptr<QuinticBezierCurve> fpdm = std::make_shared<QuinticBezierCurve>();
                fpdm->setParams(0.00, 0.00, 0.00, -Dc, -Dc, -Dc);
                double velocityDm = (fpdm->firstIntegral(1.00, 0.00) - fpdm->firstIntegral(0.00, 0.00)) * tphDm;
                double posDm = (fpdm->secondIntegral(1.00, 0.00, 0.00) - fpdm->secondIntegral(0.00, 0.00, 0.00)) * tphDm * tphDm;

                std::shared_ptr<QuinticBezierCurve> fpdp = std::make_shared<QuinticBezierCurve>();
                fpdp->setParams(0.00, 0.00, 0.00, Dc - _accel[START], Dc - _accel[START], Dc - _accel[START]);
                double velocityDp = (fpdp->firstIntegral(1.00, 0.00) - fpdp->firstIntegral(0.00, 0.00)) * tphDp;

                double pdp = (fpdp->secondIntegral(1.00, 0.00, 0.00) - fpdp->secondIntegral(0.00, 0.00, 0.00)) * tphDp * tphDp;
                double posDp = pdp + 0.50 * _accel[START] * tphDp * tphDp + _vel[START] * tphDp;

                //calculate constant deacceleration time
                //calculate time of constant deacceleration to reach 0 velocity
                double tphDc = abs((_vel[START] - (velocityDm - velocityDp)) / Dc);
                double distanceDc = (_vel[START] + velocityDp) * tphDc + 0.50 * Dc * tphDc * tphDc;
                double max_stop_distance = abs(distanceDc) + abs(posDp) + abs(posDm);

                if (in_out_time > std::numeric_limits<double>::min())
                {
                    //time is given (check safety conditions )
                    if (abs(travel_distance) <= abs(max_stop_distance))
                    {
                        in_out_time = createStopTrajectory(max_stop_distance,
                            Dc,
                            tphDp,
                            tphDm,
                            tphDc,
                            velocityDp,
                            velocityDm,
                            posDp,
                            fpdm,
                            fpdp);
                        return ResultTrajectory::SafetyStop;
                    }
                }
            }

            calculateJerkTimes(Ac, Dc, tphAp, tphAm, tphDp, tphDm);

            double aproxJerkDistAp(0.00);
            double aproxJerkDistAm(0.00);
            double aproxJerkDistDp(0.00);
            double aproxJerkDistDm(0.00);
            double aproxJerkDistance(0.00);

            double velAp(0.00);
            double velAm(0.00);
            double velDp(0.00);
            double velDm(0.00);

            calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, aproxJerkDistAp, aproxJerkDistAm, aproxJerkDistDp, aproxJerkDistDm, aproxJerkDistance, velAp, velAm, velDp, velDm);

                                  
            double p_velocity = _max_vel;
            bool reducedMaxVelocity(false);

            bool reducedMaxAccel(false);
            if (abs(travel_distance) <= 2.5 * abs(aproxJerkDistAp))
            {                    
                _use_quinticPoly = true;

                //sum of distance during jerk phases is greater then travel_distance
                //in this case we have to decrease max_acceleration
                
               
                findMaxAccelTimes(abs(travel_distance),
                                        Ac,
                                        Dc,
                                        tphAp,
                                        tphAm,
                                        tphDp,
                                        tphDm,
                                        reducedMaxAccel);

                /*In this special case where we have funny initial and final conditions and very strange constrains QuinticPolyTrajectory will be used.*/
                _use_quinticPoly = true;
                _quinticPolyTrajectory.setParameters(_pos[START], _vel[START], _accel[START], _pos[END], _vel[END], _accel[END], abs(Ac), _max_vel);                         
                double mt = tphAp + tphAm + tphDp + tphDm;

                //add poly reserve
                mt = 1.41 * mt;
                if (in_out_time > mt)
                {
                    mt = in_out_time;
                }

                _quinticPolyTrajectory.create(mt);
                in_out_time = mt;

                return QPolyTrajectory;
            }

          
            //searching for peak velocity
            if (false == findPeakVelocity(travel_distance, Ac, Dc, p_velocity))
            {
                return ResultTrajectory::NotPossible;
            }            
            reducedMaxVelocity = (p_velocity < _max_vel);


            in_out_time = createTrajectory(Ac, Dc, p_velocity);
           
            return ResultTrajectory::BTrajectory;
        }

        void  JerkLimitedTrajectory::pathFunc(double t, PathSegment* pathSegment, FUNC_PARAMS* params, double& a, double& v, double& s, double& j)
        {
            if (params->dtime > std::numeric_limits<double>::min())
            {
                double time_scale_factor = 1.00 / params->dtime;
                if (params->normalizedTime)
                {
                    double delta_t = (t - params->startTime);
                    double tau = delta_t * time_scale_factor;
                    if (params->inverseFunc)
                    {
                        j = pathSegment->getJerk(1.00 - tau);

                        a = params->f_accel - params->signA * pathSegment->getAccel(1.00 - tau);
                        v = params->f_vel + params->signVel * pathSegment->getVelocity(1.00 - tau, params->scaleVel1 * params->dtime, params->subtractInitialIntegral);
                        s = params->f_pos - params->signPos * pathSegment->getPosition(1.00 - tau, params->scalePos1 * params->dtime, params->scalePos2 * params->dtime_squared, params->subtractInitialIntegral);
                    }
                    else
                    {
                        j = params->signA *pathSegment->getJerk(tau);

                        a = params->signA * pathSegment->getAccel(tau);
                        v = params->signVel * pathSegment->getVelocity(tau, params->scaleVel1 * params->dtime, params->subtractInitialIntegral);
                        s = params->signPos * pathSegment->getPosition(tau, params->scalePos1 * params->dtime, params->scalePos2 * params->dtime_squared, params->subtractInitialIntegral);
                    }
                }
                else
                {
                    j = params->signA * pathSegment->getJerk(t);

                    a = params->signA * pathSegment->getAccel(t);
                    v = params->signVel * pathSegment->getVelocity(t, params->scaleVel1, params->subtractInitialIntegral);
                    s = params->signPos * pathSegment->getPosition(t, params->scalePos1, params->scalePos2, params->subtractInitialIntegral);
                }
            }
        }

        void JerkLimitedTrajectory::process(double t, double& position, double& velocity, double& acceleration, double& jerk)
        {
            if (_use_quinticPoly)
            {              
                _quinticPolyTrajectory.calculateValuesForTime(t, position, velocity, acceleration, jerk);
            }
            else
            {
                //find segment
                int segment = 0;
                double old = 0.00;
                for (auto& time : _times)
                {
                    if (t < (time.second + std::numeric_limits<double>::min()))
                    {
                        int si = time.first;
                        PathSegment* segment = _ptrSegments[si].get();
                        if (segment != nullptr)
                        {
                            pathFunc(t, segment, _funcParams[si].get(), acceleration, velocity, position, jerk);
                            return;
                        }
                    }
                }
            }
        }
    }
}

