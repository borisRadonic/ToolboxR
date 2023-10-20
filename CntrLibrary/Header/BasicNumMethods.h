#pragma once
#include <functional> 
#include <cmath>
#include <limits>

namespace CntrlLibrary
{
    namespace Math
    {
        namespace BasicNumMethods
        {

            enum class ResultType
            {
                Ok = 0,
                WrongInputParameters = 1,
                NotPossible = 3,
                OutOfTolerance = 5
            };

            class NewtonRaphson
            {
            public:

                NewtonRaphson() = delete;

                explicit NewtonRaphson(std::function<double(double)> function, std::function<double(double)> derivative, double tolerance = 1e-6, int maxIterations = 1000)
                    : _func(function), _dfunc(derivative), _tol(tolerance), _max_iter(maxIterations)
                {
                }

                /*only for x > 0*/
                ResultType findRoot(double initial_guess, double& root )
                {
                    if (initial_guess < 0.00)
                    {
                        root = 0.00;
                        return ResultType::WrongInputParameters;
                    }
                    double x = initial_guess;
                    for (int i = 0; i < _max_iter; ++i)
                    {
                        double df = _dfunc(x);
                        if (abs(df) > std::numeric_limits<double>::min())
                        {
                            double delta_x = _func(x) / df;
                            x -= delta_x;
                            if (x < std::numeric_limits<double>::min())
                            {
                                root = 0.00;
                                return ResultType::NotPossible;
                            }
                            if (std::abs(delta_x) < _tol)
                            {
                                root = x;
                                return ResultType::Ok;
                            }
                        }
                    }
                    root = x;  // the last computed value
                    return ResultType::OutOfTolerance;
                }

            private:
                std::function<double(double)> _func;   // Function for which root is to be found
                std::function<double(double)> _dfunc;  // Derivative of the function
                double _tol;
                int _max_iter;
            };


            class SimpsonsIntegrator
            {
            public:
                static ResultType approximateIntegral(std::function<double(double)> f, double x1, double x2, int n, double& outResult)
                {
                    if (n % 2 != 0)
                    {
                        return ResultType::WrongInputParameters;
                    }
                    double h = (x2 - x1) / n;
                    // Initialize with the boundary values
                    double integral = f(x1) + f(x2);

                    for (int i = 1; i < n; i += 2)
                    {
                        integral += 4 * f(x1 + i * h);
                    }

                    for (int i = 2; i < n - 1; i += 2)
                    {
                        integral += 2 * f(x1 + i * h);
                    }
                    if (h > std::numeric_limits<double>::min())
                    {
                        integral *= h / 3.0;
                    }
                    else
                    {
                        return ResultType::NotPossible;
                    }
                    outResult = integral;
                    return ResultType::Ok;
                }
            };
        }
    }
}


