#pragma once
#include <functional> 
#include <cmath>
#include <limits>
#include "MathFunctionBase.h"

namespace CntrlLibrary
{
    namespace Math
    {
        class LinearFunction : public MathFunctionBase
        {
        private:
            double _m; // Slope
            double _b; // Y-intercept

        public:
            LinearFunction(double slope, double yIntercept) : _m(slope), _b(yIntercept)
            {
            }

            inline virtual double compute(double t) override
            {
                return _m * t + _b; // Linear equation
            }

            inline virtual double firstDerivative(double /* t */) override
            {
                return _m;
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

            inline virtual double firstIntegral(double t, double c1 = 0.00) override
            {
                return( 0.5 * _m * t * t + _b * t + c1);
            }

            inline virtual double secondIntegral(double t, double c1 = 0.00, double c2 = 0.00) override
            {
                return( (0.5/3.0) * _m * t * t * t + 0.50 * _b * t * t + c1 * t + c2);
            }
        };
    }
}

