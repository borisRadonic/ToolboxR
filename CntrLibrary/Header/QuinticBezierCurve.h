﻿
#pragma once
#include <functional> 
#include <cmath>
#include <limits>
#include "BasicNumMethods.h"
#include <Eigen/Dense>

namespace CntrlLibrary
{
    namespace Math
    {
        namespace Bezier
        {
            class QuinticBezierCurve
            {
            public:
                             

                QuinticBezierCurve()
                {
                }

                QuinticBezierCurve(const QuinticBezierCurve& rhs)
                    : _P0(rhs._P0)
                    , _P1(rhs._P1)
                    , _P2(rhs._P2)
                    , _P3(rhs._P3)
                    , _P4(rhs._P4)
                    , _P5(rhs._P5)
                {
                } 

                inline double calculateIntegral(double t)
                {
                    double t2 = t * t;
                    double t3 = t2 * t;
                    double t4 = t3 * t;
                    double t5 = t4 * t;
                    double t6 = t5 * t;
                    
                    
                    double t_2 = (1 - t) * (1 - t);
                    double t_3 = t_2 * (1 - t);
                    double t_4 = t_3 * (1 - t);
                    double t_5 = t_4 * (1 - t);
                    double t_6 = t_5 * (1 - t);
                  

                    return ( -(1.00 / 6.00) * _P0 * t_6
                        + 5.00 * _P1 * ( (t6 / 6.00) - 4.00 * (t5 / 5.00) + 3.00 * (t4 / 2.00) - 4.0 * (t3 / 3.00) + (t2 / 2.00))
                        - 10.00 * _P2 * (t6 / 6.00 - 3.00 * (t5 / 5.00) + 3.00 * (t4 / 4.00) - (t3 / 3.00))
                        + 10.00 * _P3 * (t6 / 6.00 - 2.00 * (t5 / 5.00) + (t4 / 4.00))
                        - 5.00 * _P4 * (t6 / 6.00 - t5 / 5.00)
                        + _P5 * t6 / 6.00);
                }


                inline double calculateSecondIntegral(double t)
                {
                    double t2 = t * t;
                    double t3 = t2 * t;
                    double t4 = t3 * t;
                    double t5 = t4 * t;
                    double t6 = t5 * t;
                    double t7 = t6 * t;

                    double t_2 = (1 - t) * (1 - t);
                    double t_3 = t_2 * (1 - t);
                    double t_4 = t_3 * (1 - t);
                    double t_5 = t_4 * (1 - t);
                    double t_6 = t_5 * (1 - t);
                    double t_7 = t_6 * (1 - t);

                    return (-(1.00 / 42.00) * _P0 * t_7
                        + 0.833333 * _P1 * (0.142857 * t7 - 0.80 * t6 + 1.80 * t5 - 2.00 * t4 + t3)
                        - 1.66667 * _P2 * (0.142857 * t7 - 0.60 * t6 + 0.90 * t5 - 0.50 * t4)
                        + 1.66667 * _P3 * (0.142857 * t7 - 0.40 * t6 + 0.30 * t5)
                        + (1.00/6.00) * _P4 * (t6 - 5.00 * (t7/7.00) )
                        + _P5 * t7 / 42.00);
                }


                inline double calculateX(double t)
                {
                    double t2 = t * t;
                    double t3 = t2 * t;
                    double t4 = t3 * t;
                    double t5 = t4 * t;
                    double t_2 = (1 - t) * (1 - t);
                    double t_3 = t_2 * (1 - t);
                    double t_4 = t_3 * (1 - t);
                    double t_5 = t_4 * (1 - t);
                    return (_P0 * t_5
                        + 5.00 * _P1 * t_4 * t
                        + 10.00 * t_3 * t2 * _P2
                        + 10.00 * _P3 * t_2 * t3
                        + 5.00 * (1 - t) * t4 * _P4
                        + t5 * _P5);
                }

                inline double calculateDerX(double t)
                {
                    double t2 = t * t;
                    double t3 = t2 * t;
                    double t4 = t3 * t;
                    double t_2 = (1 - t) * (1 - t);
                    double t_3 = t_2 * (1 - t);
                    double t_4 = t_3 * (1 - t);
            
                    return (5.00 * t_4 * (_P1 - _P0)
                        + 20.00 * t_3 * t * (_P2 - _P1)
                        + 30.00 * t_2 * t2 * (_P3 - _P2)
                        + 20.00 * (1 - t) * t3 * (_P4 - _P3)
                        + 5.00 * t4 * (_P5 - _P4));
                }

                inline double calculateDerX2(double t)
                {
                    double t2 = t * t;
                    double t3 = t2 * t;
                    double t_2 = (1 - t) * (1 - t);
                    double t_3 = t_2 * (1 - t);
                    return (20.00 * t_3 * (_P2 - 2.00 * _P1 + _P0)
                        + 60 * t_2 * t * (_P3 - 2.00 * _P2 + _P1)
                        + 60.00 * (1 - t) * t2 * (_P4 - 2.00 * _P3 + _P2)
                        + 20.00 * t3 * (_P5 - _P4));
                }

                inline double calculateTimeScaledFirstDer(double scale, double t)
                {
                    double scaled_t = t / scale;  // Scale the input time

                    if (scaled_t < 0.0) scaled_t = 0.0;
                    if (scaled_t > 1.0) scaled_t = 1.0;


                    double t2 = scaled_t * scaled_t;
                    double t3 = t2 * scaled_t;
                    double t4 = t3 * scaled_t;
                    double t_2 = (1 - scaled_t) * (1 - scaled_t);
                    double t_3 = t_2 * (1 - scaled_t);
                    double t_4 = t_3 * (1 - scaled_t);

                    // Multiply the result by the scale to adjust the rate of change
                    return (5.00 * t_4 * (_P1 - _P0)
                        + 20.00 * t_3 * scaled_t * (_P2 - _P1)
                        + 30.00 * t_2 * t2 * (_P3 - _P2)
                        + 20.00 * (1 - scaled_t) * t3 * (_P4 - _P3)
                        + 5.00 * t4 * (_P5 - _P4)) / scale;
                }

                inline double calculateTimeScaledSecondDerX(double _P0, double _P1, double _P2, double _P3, double _P4, double _P5, double scale, double t)
                {
                    double scaled_t = t / scale;

                    // Ensure scaled_t is within the [0, 1] interval
                    if (scaled_t < 0.0) scaled_t = 0.0;
                    if (scaled_t > 1.0) scaled_t = 1.0;

                    double t2 = scaled_t * scaled_t;
                    double t3 = t2 * scaled_t;
                    double t_2 = (1 - scaled_t) * (1 - scaled_t);
                    double t_3 = t_2 * (1 - scaled_t);

                    // Calculate the second derivative and divide by the square of the scale
                    return (20.00 * t_3 * (_P2 - 2.00 * _P1 + _P0)
                        + 60.00 * t_2 * scaled_t * (_P3 - 2.00 * _P2 + _P1)
                        + 60.00 * (1 - scaled_t) * t2 * (_P4 - 2.00 * _P3 + _P2)
                        + 20.00 * t3 * (_P5 - 2.00 * _P4 + _P3)) / (scale * scale);
                }

                inline double calculateTimeScaledThirdDerX(double _P0, double _P1, double _P2, double _P3, double _P4, double _P5, double scale, double t)
                {
                    double scaled_t = t / scale;

                    // Ensure scaled_t is within the [0, 1] interval
                    if (scaled_t < 0.0) scaled_t = 0.0;
                    if (scaled_t > 1.0) scaled_t = 1.0;

                    double t2 = scaled_t * scaled_t;

                    // Calculate the third derivative and divide by the cube of the scale
                    return (60.00 * (1 - scaled_t) * t2 * (_P5 - 3.00 * _P4 + 3.00 * _P3 - _P2)
                        + 60.00 * t2 * scaled_t * (_P4 - 3.00 * _P3 + 3.00 * _P2 - _P1)
                        + 20.00 * scaled_t * scaled_t * (_P3 - 3.00 * _P2 + 3.00 * _P1 - _P0)) / (scale * scale * scale);
                }

                Math::BasicNumMethods::ResultType findFirstDerRoots(std::vector<double>& roots)
                {
                    double d1 = _P2 - 2.00 * _P1 + _P0;
                    double d2 = _P3 - 2.00 * _P2 + _P1;
                    double d3 = _P4 - 2.00 * _P3 + _P2;

                    double A = 20.00 * d1 + 60.00 * (d2 - d3) +  20.00  * (_P5 - _P4);
                    double B = 60.00 * (d1 + d3) - 120.00 * d2;
                    double C = 60.00 * (d1 + d2);
                    double D = -20.00 * d1;
                   
                    if ( (abs(A) <= 0.0001) && (abs(B) <= 0.0001) && (abs(C) > 0.0001))
                    {
                        roots.push_back( -D / C );
                        return Math::BasicNumMethods::ResultType::Ok;

                    }
                    else if ( (abs(A) <= 0.0001) && (abs(B) >= 0.0001) )
                    {
                        /*we have only two roots and we can use quadratic formula*/
                        double discriminant = C * C - 4.00 * B * D;
                        if (discriminant > 0.00)
                        {
                            double root1 = (-C + std::sqrt(discriminant)) / (2.0 * B);
                            double root2 = (-C - std::sqrt(discriminant)) / (2.0 * B);
                            
                            if (((root1 >= 0.00) && (root1 <= 1.00)) )
                            {
                                roots.push_back(root1);
                            }
                            if (((root2 >= 0.00) && (root2 <= 1.00)))
                            {
                                roots.push_back(root2);
                            }
                            return Math::BasicNumMethods::ResultType::Ok;
                        }
                        else
                        {
                            return Math::BasicNumMethods::ResultType::NotPossible;
                        }
                    }
                    else
                    {

                        // Construct the companion matrix
                        Eigen::Matrix3cd companionMatrix = Eigen::Matrix3cd::Zero();
                        companionMatrix(0, 1) = 1.0;
                        companionMatrix(1, 2) = 1.0;

                        companionMatrix(2, 0) = -D / A;
                        companionMatrix(2, 1) = -C / A;
                        companionMatrix(2, 2) = -B / A;

                        // Solve for eigenvalues
                        Eigen::ComplexEigenSolver<Eigen::Matrix3cd> solver(companionMatrix);
                        Eigen::Vector3cd eigenValues = solver.eigenvalues();

                        for (auto& r : eigenValues)
                        {
                            if ( (abs( r.imag() ) <  0.0001) && ( r.real() >= 0.00) )
                            {
                                roots.push_back(r.real());
                            }
                        }
                    }
                    if (roots.size() > 0)
                    {
                        return Math::BasicNumMethods::ResultType::Ok;
                    }
                    return Math::BasicNumMethods::ResultType::NotPossible;
                }

                inline double calculateDerX3(double t)
                {
                    double t2 = t * t;
                    double t_2 = (1 - t) * (1 - t);   
                    return (60.00 * t_2 * (_P3 - 3.00 * _P2 + 3.00 * _P1 - _P0)
                        + 120 * (1 - t) * t * (_P4 - 3.00 * _P3 + 3.00 * _P2 - _P1)
                        + 60.00 * t2 * (_P5 - _P4));
                }

                inline void calculate(double t, double& x, double& x_der1, double& x_der2, double& x_der3)
                {
                    double t2 = t*t;
                    double t3 = t2 * t;
                    double t4 = t3 * t;
                    double t5 = t4 * t;

                    double t_2 = (1 - t) * (1-t);
                    double t_3 = t_2 * (1 - t);
                    double t_4 = t_3 * (1 - t);
                    double t_5 = t_4 * (1 - t);

                    x = _P0 * t_5
                      + 5.00 * _P1 * t_4 * t
                      + 10.00 * t_3 * t2 *_P2
                      + 10.00 * _P3 * t_2 * t3
                      + 5.00 * (1 - t) * t4 * _P4
                      + t5 * _P5;

                    x_der1 = 5.00 * t_4 * (_P1 - _P0)
                           + 20.00 * t_3 * t * (_P2 - _P1)
                           + 30.00 * t_2 * t2 * (_P3 - _P2)
                           + 20.00 * (1 - t) * t3 * (_P4 - _P3)
                           + 5.00 * t4 * (_P5 - _P4);

                    x_der2 = 20.00 * t_3 * (_P2 - 2.00 * _P1 + _P0)
                        + 60 * t_2 * t * (_P3 - 2.00 * _P2 + _P1)
                        + 60.00 * (1 - t) * t2 * (_P4 - 2.00 * _P3 + _P2)
                        + 20.00 * t3 * (_P5 - _P4);

                    x_der3 =  60.00 * t_2 * (_P3 - 3.00 * _P2 + 3.00 * _P1 - _P0)
                             + 120 * (1 - t) * t * (_P4 - 3.00 * _P3 + 3.00 * _P2 - _P1)
                             + 60.00 * t2 * (_P5 - _P4);
                }

                inline void setParams(double p0, double p1, double p2, double p3, double p4, double p5)
                {
                    _P0 = p0;
                    _P1 = p1;
                    _P2 = p2;
                    _P3 = p3;
                    _P4 = p4;
                    _P5 = p5;
                }

                inline double getP0() const
                {
                    return _P0;
                }

                inline double getP1() const
                {
                    return _P1;
                }

                inline double getP2() const
                {
                    return _P2;
                }

                inline double getP3() const
                {
                    return _P3;
                }

                inline double getP4() const
                {
                    return _P4;
                }

                inline double getP5() const
                {
                    return _P5;
                }

            private:

                const double MIN_TIME_DIFFERENCE = 1e-5; /*10 us*/
                

                double _P0 = 0.00;
                double _P1 = 0.00;
                double _P2 = 0.00;
                double _P3 = 0.00;
                double _P4 = 0.00;
                double _P5 = 0.00;
            };
        }
    }
}



