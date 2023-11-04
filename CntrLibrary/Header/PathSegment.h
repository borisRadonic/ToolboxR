#pragma once
#include <vector>
#include <memory>
#include "MathFunctionBase.h"

using namespace CntrlLibrary::Math;

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {

        /*Every path segment is based on Acceleration. Jerk is derivative and all other functions a integrals.*/
        class PathSegment
        {

        public:

            PathSegment(    double startTime,
                            double endTime,
                            double startAccel,
                            double startVel,
                            double startPos,
                            std::shared_ptr<MathFunctionBase> mathFunc)
                : _mathFunction(mathFunc)
                , _startTime(startTime)
                , _endTime(endTime)
                , _startAccel(startAccel)
                , _startVel(startVel)
                , _startPos(startPos)
            {                 
                _firstIntEnd = _mathFunction->firstIntegral(_endTime, 0.00);
                _secondIntEnd = _mathFunction->secondIntegral(_endTime, 0.00, 0.00);
                _firstIntStart = _mathFunction->firstIntegral(_startTime, 0.00);
                _secondIntStart = _mathFunction->secondIntegral(_startTime, 0.00, 0.00);
            }

            double getAccel(double t)
            {
              
                return (_startAccel + _mathFunction->compute(getLocalTime(t)));
            }

            double getVelocity(double t, double scaleInt = 1.00, bool subtractInitialIntegral = false )
            {
                if (subtractInitialIntegral)
                {
                    return (_startVel + scaleInt * ( _mathFunction->firstIntegral(getLocalTime(t), 0.00 ) - _firstIntStart) );
                }
                else
                {
                    return ( _startVel + scaleInt * _mathFunction->firstIntegral(getLocalTime(t), 0.00 ) );
                }
                
            }

            double getPosition(double t, double scale1 = 1.00, double scale2 = 1.00, bool subtractInitialIntegral = false)
            {
                double lt = getLocalTime(t);
                if (subtractInitialIntegral)
                {
                    //double c1 = _firstIntEnd * lt * scale1;
                    return ( _startPos  + _startVel * lt * scale1 + scale2 * (_mathFunction->secondIntegral(lt, 0.00, 0.00) - _secondIntStart) );
                }
                else
                {
                    return ( _startPos + _startVel * lt * scale1 + scale2 * _mathFunction->secondIntegral(lt, 0.00, 0.00) );
                }
            }

            double getEndVelocity()
            {
                return (_firstIntEnd + _startVel);
            }

            double getEndPositiony()
            {
                return (_secondIntEnd + _startPos);
            }

            double getFirstIntegralAtStart()
            {
                return (_firstIntStart);
            }

            double geSecondIntegralAtStart()
            {
                return (_secondIntStart);
            }

            double geFirstIntegralAtEnd()
            {
                return (_firstIntEnd);
            }

            double geSecondIntegralAtEnd()
            {
                return (_secondIntEnd);
            }
                      

        private:

            inline double getLocalTime(double t) const
            {
                if (t < _startTime)
                {
                    return _startTime;
                }

                if (t > _endTime)
                {
                    return _endTime;
                }
                return (t - _startTime);
            }

        private:

            std::shared_ptr<MathFunctionBase> _mathFunction;
            
            double _startTime       = 0.00;
            double _endTime         = 1.00;
            double _startAccel      = 0.00;
            double _startVel        = 0.00;
            double _startPos        = 0.00;
            double _firstIntEnd     = 0.00;
            double _secondIntEnd    = 0.00;
            double _firstIntStart    = 0.00;
            double _secondIntStart = 0.00;
        };

    }
}


