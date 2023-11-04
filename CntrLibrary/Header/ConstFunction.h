
#pragma once
#include <functional> 
#include <cmath>
#include <limits>
#include "MathFunctionBase.h"

namespace CntrlLibrary
{
    namespace Math
    {
        class ConstFunction : public MathFunctionBase
        {
        

        public:
            ConstFunction(double constantValue) : _c(constantValue)
            {
            }

            inline virtual double compute(double t) override
            {
                return _c; //// Constant value regardless of t
            }

            inline virtual double firstDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double secondDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double thirdDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double fourthDerivative(double /* t */) override
            {
                return 0.00;
            }

            inline virtual double firstIntegral(double t, double c1 = 0.00 ) override
            {
                return(_c * t + c1);
            }

            inline virtual double secondIntegral(double t, double c1 = 0.00, double c2 = 0.00) override
            {
                return( 0.5 * _c * t * t + c1 *t + c2);
            }

        private:
            double _c; // Constant value
        };
    }
}
