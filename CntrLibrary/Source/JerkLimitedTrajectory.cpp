
#include "JerkLimitedTrajectory.h"
#include "ConstFunction.h"
#include <cmath>
#include <algorithm>

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
            tphAp = abs(Ac - _accel[START]) / _max_jerk; //max. duration of + jerk phase
            tphAm = abs(Ac) / _max_jerk; //max. duration of - jerk phase
            tphDp = abs(Dc) / _max_jerk;  //max. duration of + jerk phase for deacceleration
            tphDm = (abs(Dc) - _accel[END]) / _max_jerk; //max. duration of - jerk phase for deacceleration
        }

        void JerkLimitedTrajectory::calculateJerkDistances( double Ac,
                                                            double Dc,
                                                            double& tphAp,
                                                            double& tphAm,
                                                            double& tphDp,                
                                                            double& tphDm,
                                                            double& aproxJerkDistAp,
                                                            double& aproxJerkDistAm,
                                                            double& aproxJerkDistDp,
                                                            double& aproxJerkDistDm,
                                                            double& aproxJerkDistance)
        {
            //during the jerk phase average acceleration is 50% of max ( for our Bazier sigmoid function ) -> aproximation
            aproxJerkDistAp = abs(0.166 * (Ac - _accel[START]) * tphAp * tphAp) + abs(tphAp * _vel[START]);
            aproxJerkDistAm = abs(0.166 * Ac * tphAm * tphAm);
            aproxJerkDistDp = abs(0.166 * (Dc - _accel[START]) * tphDp * tphDp);
            aproxJerkDistDm = abs(0.166 * (abs(Dc) - _accel[END]) * tphDm * tphDm) + abs(tphDm * _vel[END]);
            aproxJerkDistance = aproxJerkDistAp + aproxJerkDistAm + aproxJerkDistDp + aproxJerkDistDm;
        }

        bool JerkLimitedTrajectory::findNewMaxVelocity( double travel_distance,
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
                                                        double& aproxJerkDistance)
        {
            //find appropriate velocity
            const int MAX_STEPS = 1000;
            double min_v = 0.00;
            double max_v = _max_vel;
            bool reduced(false);
            uint32_t counter(0);
            while (counter < MAX_STEPS)
            {
                counter++;
                calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, aproxJerkDistAp, aproxJerkDistAm, aproxJerkDistDp, aproxJerkDistDm, aproxJerkDistance);
                double difference = travel_distance - abs(aproxJerkDistance);
                if (difference <= 0.00)
                {
                    reduced = true;
                    max_v = new_max_velocity;
                    new_max_velocity = (min_v + max_v) / 2.00;
                }
                else
                {
                    if (abs(difference) <= abs(_max_pos_error))
                    {
                        return true;
                    }
                    if (reduced)
                    {
                        min_v = new_max_velocity;
                        new_max_velocity = (min_v + max_v) / 2.00;
                    }
                }
            }
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

            reduced = false;
            uint32_t counter(0);

            double aproxJerkDistAp(0.00);
            double aproxJerkDistAm(0.00);
            double aproxJerkDistDp(0.00);
            double aproxJerkDistDm(0.00);
            double aproxJerkDistance(0.00);

            while (counter < MAX_STEPS)
            {
                counter++;
                calculateJerkTimes(Ac, Dc, tphAp, tphAm, tphDp, tphDm);
                calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, aproxJerkDistAp, aproxJerkDistAm, aproxJerkDistDp, aproxJerkDistDm, aproxJerkDistance);

                //impact of velocity...
                double velocityAp = _vel[START] + 0.50 * (Ac - _accel[START]) * tphAp;
                double velocityAm = _vel[START] + 0.50 * (Ac - _accel[START]) * tphAm;

                aproxJerkDistAm += abs(tphAm * velocityAp);
                aproxJerkDistDp += abs(tphDp * velocityAm);

                aproxJerkDistance = aproxJerkDistAp + aproxJerkDistAm + aproxJerkDistDp + aproxJerkDistDm;
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
            double tphDp = _times[AMINUS]- startTime;
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
            _funcParams[AMINUS]->dtime = tphDp;
            _funcParams[AMINUS]->dtime_squared = tphDp * tphDp;
            _funcParams[AMINUS]->f_accel = 0.00;
            _funcParams[AMINUS]->f_vel = 0.00;
            _funcParams[AMINUS]->f_pos = 0.00;
        }


        void JerkLimitedTrajectory::addDMinusFuncParams(double startTime)
        {
            double tphDm = _times[DMINUS] - startTime;
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
            _funcParams[DPLUS]->startTime = 0.00;
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
            _durations[DPLUS] = tphDp;            
            _vel[DPLUS] = _vel[START] + velocityDp;
            _pos[DPLUS] = posDp;
                       
            _functions[DMINUS] = fpdm;
            _vel[DMINUS] = velocityDm;

            _ptrSegments[DPLUS]->create(0.00, 1.00, _accel[START], _vel[START], _pos[START], _functions[DPLUS]);

            _durations[DCONST] = tphDc;
            //time at the end of D const
            _times[DCONST] = _times[DPLUS] + tphDc;

            _durations[DMINUS] = tphDm;
            
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
            addDMinusFuncParams(_times[DCONST]);

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
            _durations.clear();
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
            clearMaps();
           
            double travel_distance = _pos[END] - _pos[START];

            if ( abs(travel_distance) <= std::numeric_limits<double>::min())
            {
                return ResultTrajectory::NotPossible;
            }
           
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

        
            /*effect of jerk phases*/
            double Tjv      = abs(Ac / _max_jerk);
            double Tjd      = abs((Dc - _accel[START]) / _max_jerk);

            double tphAp(0.00);
            double tphAm(0.00);
            double tphDp(0.00);
            double tphDm(0.00);


          
            /*check if safety stop required*/
            tphDp = abs( Dc - _accel[START] ) / _max_jerk; 
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
                if (abs(travel_distance) <= abs(max_stop_distance) )
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

            calculateJerkTimes(Ac, Dc, tphAp, tphAm, tphDp, tphDm);

            double aproxJerkDistAp(0.00);
            double aproxJerkDistAm(0.00);
            double aproxJerkDistDp(0.00);
            double aproxJerkDistDm(0.00);
            double aproxJerkDistance(0.00);

            calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, aproxJerkDistAp, aproxJerkDistAm, aproxJerkDistDp, aproxJerkDistDm, aproxJerkDistance);


            tphAp = abs(Ac - _accel[START]) / _max_jerk; //max. duration of + jerk phase
            tphAm = abs(Ac) / _max_jerk; //max. duration of - jerk phase

            bool reducedMaxAccel(false);
            if (abs(travel_distance) <= abs(aproxJerkDistAp))
            {                    
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

                    //recalculate jerk distances
                    calculateJerkDistances(Ac, Dc, tphAp, tphAm, tphDp, tphDm, aproxJerkDistAp, aproxJerkDistAm, aproxJerkDistDp, aproxJerkDistDm, aproxJerkDistance);
            }

            //add velocity impact (in case of max velocity
            /*during A- and D+ velocity ist nearly Vc (we do not know Vc), but we take max_v*/
            aproxJerkDistAm += abs(tphAm * _max_vel);
            aproxJerkDistDp += abs(tphDp * _max_vel);
            double new_max_velocity = _max_vel;

            aproxJerkDistance = aproxJerkDistAp + aproxJerkDistAm + aproxJerkDistDp + aproxJerkDistDm;

            bool reducedMaxVelocity(false);
            if (abs(travel_distance) < abs(aproxJerkDistance))
            {
                reducedMaxVelocity = true;
                if (false == findNewMaxVelocity(travel_distance,
                    Ac,
                    Dc,
                    new_max_velocity,
                    aproxJerkDistAp,
                    tphAp,
                    tphAm,
                    tphDp,
                    tphDm,
                    aproxJerkDistAm,
                    aproxJerkDistDp,
                    aproxJerkDistDm,
                    aproxJerkDistance))
                {
                    return ResultTrajectory::NotPossible;
                }
            }               
            double max_velocity_squ = new_max_velocity * new_max_velocity;
            double min_max_vel_distance = abs((max_velocity_squ - start_velocity_squ) / (abs(Ac)+abs(Dc)) + max_velocity_squ / (abs(Ac) + abs(Dc)));
            min_max_vel_distance += abs(aproxJerkDistance);


            ///TODO!!!!!!!
            //check if the velocity exceeded max_velocity - safety condition2
            if (_vel[START] > _max_vel)
            {
                ///TODO!!!!!!!
                //deaccelerate to _max_velocity
                double dvel = abs(_vel[START]) - _max_vel;
                double max_reduce_vel_distance = (dvel * dvel) / (2.00 * _max_accel);
                //deacceleration...
            }
            ///TODO!!!!!!!

            if (reducedMaxAccel)
            {                   
                //we have only jerk phases               
                _times[APLUS]   = tphAp;
                _times[AMINUS]  = tphAp + tphAm;
                _times[DPLUS]   = _times[AMINUS] + tphDp;
                _times[DMINUS]  = _times[DPLUS] + tphDm;

                _durations[APLUS]   = tphAp;
                _durations[AMINUS]  = tphAm;
                _durations[DPLUS]   = tphDp;
                _durations[DMINUS]  = tphDm;
            }
            else
            {
                if (reducedMaxVelocity)
                {
                    if( abs(travel_distance) < min_max_vel_distance )
                    {
                        //triangular profile
                        if ((travel_distance - aproxJerkDistance) > 0.00)
                        {
                            Vc = sign * sqrt((2.00 * max_velocity_squ * (travel_distance - aproxJerkDistance) - Ac * start_velocity_squ) / (Ac + abs(Dc)));
                        }
                        if (Vc < new_max_velocity)
                        {
                            Vc = sign * new_max_velocity;
                        }
                     
                        _times[APLUS] = tphAp;
                        //_times[ACONST] = _times[APLUS] + 
                         _times[DPLUS] = _times[ACONST] + tphAm;
                        //_times[DCONST] = _times[DPLUS] + 
                        _times[DMINUS] = _times[DCONST] + tphDp;
                      
                    }
                    else
                    {
                        //can not be TODO
                    }
                }
                else
                {
                    
                    if (abs(travel_distance) < min_max_vel_distance)
                    {
                        //triangular profile
                        Vc = sign * sqrt((2.00 * max_velocity_squ * (travel_distance - aproxJerkDistance) - Ac * start_velocity_squ) / (Ac + abs(Dc)));
                    }
                    else
                    {
                        Vc = sign * _max_vel;
                    }
                    //we do not have const velocity phase                       
                 
                    //_times[DPLUS] = _times[APLUS] + tphAm;
                   //_times[DMINUS] = _times[DPLUS] + tphDp;
                    //_times[END] = _times[DMINUS] + tphDm;
                }
            }

            if (in_out_time <= std::numeric_limits<double>::min())
            {
                //Min time must be calculated
                //Ta = Ta + tphAp + tphAm;
                //Td = Td + tphDp + tphDm;
            }

            return ResultTrajectory::NotPossible;
        }

        void  JerkLimitedTrajectory::pathFunc(double t, PathSegment* pathSegment, FUNC_PARAMS* params, double& a, double& v, double& s)
        {
            double time_scale_factor = 1.00 / params->dtime;
           
            if (params->normalizedTime)
            {
                double delta_t = (t - params->startTime);
                double tau = delta_t * time_scale_factor;
                if (params->inverseFunc)
                {
                    a = params->f_accel - params->signA * pathSegment->getAccel(1.00 - tau);
                    v = params->f_vel + params->signVel * pathSegment->getVelocity(1.00 - tau, params->scaleVel1 * params->dtime, params->subtractInitialIntegral);
                    s = params->f_pos - params->signPos * pathSegment->getPosition(1.00 - tau, params->scalePos1 * params->dtime, params->scalePos2 * params->dtime_squared, params->subtractInitialIntegral);
                }
                else
                {
                    a = params->signA * pathSegment->getAccel(tau);
                    v = params->signVel * pathSegment->getVelocity(tau, params->scaleVel1 * params->dtime, params->subtractInitialIntegral);
                    s = params->signPos * pathSegment->getPosition(tau, params->scalePos1 * params->dtime, params->scalePos2 * params->dtime_squared, params->subtractInitialIntegral);
                }
            }
            else
            {
                a = params->signA * pathSegment->getAccel(t);
                v = params->signVel * pathSegment->getVelocity(t, params->scaleVel1, params->subtractInitialIntegral);
                s = params->signPos * pathSegment->getPosition(t, params->scalePos1, params->scalePos2, params->subtractInitialIntegral);
            }
        }

        void JerkLimitedTrajectory::process(double t, double& position, double& velocity, double& acceleration)
        {
            //find segment
            int segment = 0;
            double old = 0.00;
            for (auto& time : _times)
            {
                if( t < time.second + std::numeric_limits<double>::min() )
                {
                    int si = time.first;
                    PathSegment* segment = _ptrSegments[si].get();
                    if (segment != nullptr)
                    {
                        pathFunc( t, segment, _funcParams[si].get(), acceleration, velocity, position );
                        return;
                    }
                }
            }
        }
    }
}

