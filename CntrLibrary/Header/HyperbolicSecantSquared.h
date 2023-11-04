#pragma once
#include <cmath>
#include <limits>
#include "MathFunctionBase.h"

#ifndef NO_EXCEPTION
#include<stdexcept>
#endif

namespace CntrlLibrary
{
    namespace Math
    {
        class HyperbolicSecantSquared : public MathFunctionBase
        {
       
        public:

            HyperbolicSecantSquared(double a):_a(a)
            {
            }

            inline virtual double compute(double t) override
            {
                 return _a * std::pow(sech(t), 2.00);
            }

            inline virtual double firstDerivative(double t) override
            {
                return ( - 2.00 * std::pow(sech(t), 2.00) * std::tanh(t) );
            }

            inline virtual double secondDerivative(double t) override
            {
                return (-4.00 * std::pow(sech(t), 2.00) * std::pow(tanh(t), 2.00) - 2.00 * std::pow(sech(t), 4.00));;
            }

            inline virtual double thirdDerivative(double /* t */) override
            {
                #ifndef NO_EXCEPTION
                throw std::runtime_error("fourthIntegral not implemented");
                #else
                return 0.00;
                #endif
            }

            inline virtual double fourthDerivative(double /* t */) override
            {
                #ifndef NO_EXCEPTION
                throw std::runtime_error("fourthIntegral not implemented");
                #else
                return 0.00;
                #endif        
            }

            inline virtual double firstIntegral(double t, double c1 = 0.00) override
            {
                return(std::tanh(t) + c1);
            }

            inline virtual double secondIntegral(double t, double c1 = 0.00, double c2 = 0.00) override
            {
                return(_a * std::log( abs( cosh(t) ) ) + c1 * t + c2);
            }

        private:
            double sech(double x) const
            {
                // Hyperbolic secant is 1 / cosh(x)
                return 1.0 / std::cosh(x);
            }

        private:
            double _a; //amplitude
        };
    }
}

