
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

            inline virtual double firstIntegral(double t) override
            {
                return(_c * t);
            }

            inline virtual double secondIntegral(double t) override
            {
                return( 0.5 * _c * t * t );
            }

        private:
            double _c; // Constant value
        };
    }
}
