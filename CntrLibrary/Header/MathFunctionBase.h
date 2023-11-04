#pragma once

namespace CntrlLibrary
{
    namespace Math
    {
        class MathFunctionBase
        {
        public:
            
            virtual double compute(double t)            = 0;

            virtual double firstDerivative(double t)    = 0;

            virtual double secondDerivative(double t)   = 0;

            virtual double thirdDerivative(double t)    = 0;

            virtual double fourthDerivative(double t)   = 0;

            virtual double firstIntegral(double t, double c1) = 0;

            virtual double secondIntegral(double t, double c1, double c2)     = 0;
            
            virtual ~MathFunctionBase() {}
        };
    }
}
