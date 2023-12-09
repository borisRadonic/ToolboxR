/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

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

            PathSegment()
            {
            }

            void create(double startTime,
                        double endTime,
                        double startAccel,
                        double startVel,
                        double startPos,
                        std::shared_ptr<MathFunctionBase> mathFunc)
            {

                _mathFunction = mathFunc;
                _startTime = startTime;
                _endTime = endTime;
                _startAccel = startAccel;
                _startVel = startVel;
                _startPos = startPos;
                _firstIntEnd = _mathFunction->firstIntegral(_endTime, 0.00);
                _secondIntEnd = _mathFunction->secondIntegral(_endTime, 0.00, 0.00);
                _firstIntStart = _mathFunction->firstIntegral(_startTime, 0.00);
                _secondIntStart = _mathFunction->secondIntegral(_startTime, 0.00, 0.00);
                _created = true;
            }

            double getJerk(double t)
            {
                if (_created)
                {
                    return (_mathFunction->firstDerivative(getLocalTime(t)));
                }
                return 0.00;
            }

            double getAccel(double t)
            {
                if (_created)
                {
                    return (_startAccel + _mathFunction->compute(getLocalTime(t)));
                }
                return 0.00;
            }            

            double getVelocity(double t, double scaleInt = 1.00, bool subtractInitialIntegral = false )
            {
                if (_created)
                {
                    if (subtractInitialIntegral)
                    {
                        return (_startVel + scaleInt * (_mathFunction->firstIntegral(getLocalTime(t), 0.00) - _firstIntStart));
                    }
                    else
                    {
                        return (_startVel + scaleInt * _mathFunction->firstIntegral(getLocalTime(t), 0.00));
                    }
                }
                return 0.00;
            }

            double getPosition(double t, double scale1 = 1.00, double scale2 = 1.00, bool subtractInitialIntegral = false)
            {
                if (_created)
                {
                    double lt = getLocalTime(t);
                    if (subtractInitialIntegral)
                    {
                        double c1 = _firstIntStart * scale2 * lt;
                        return (_startPos + _startVel * lt * scale1 + scale2 * (_mathFunction->secondIntegral(lt, 0.00, 0.00) - _secondIntStart) - c1);
                    }
                    else
                    {
                        return (_startPos + _startVel * lt * scale1 + scale2 * _mathFunction->secondIntegral(lt, 0.00, 0.00));
                    }
                }
                return 0.00;
            }

            double getEndVelocity()
            {
                if (_created)
                {
                    return (_firstIntEnd + _startVel);
                }
                return 0.00;
            }

            double getEndPositiony()
            {
                if (_created)
                {
                    return (_secondIntEnd + _startPos);
                }
                return 0.00;
            }

            double getFirstIntegralAtStart()
            {
                if (_created)
                {
                    return (_firstIntStart);
                }
                return 0.00;
            }

            double geSecondIntegralAtStart()
            {
                if (_created)
                {
                    return (_secondIntStart);
                }
                return 0.00;
            }

            double geFirstIntegralAtEnd()
            {
                if (_created)
                {
                    return (_firstIntEnd);
                }
                return 0.00;
            }

            double geSecondIntegralAtEnd()
            {
                if (_created)
                {
                    return (_secondIntEnd);
                }
                return 0.00;
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
            bool _created = false;
        };

    }
}


