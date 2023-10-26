
#pragma once
#include <functional> 
#include <cmath>
#include <limits>
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

                inline double calculateScalDerX(double scale, double t)
                {
                    double s = scale;
                    double s_t = (s - t);
                    double s_t2 = s_t * s_t;
                    double s_t3 = s_t2 * s_t;
                    double s_t4 = s_t3 * s_t;
                    double s_t5 = s_t4 * s_t;

                    return (-5.00 * _P0 * s_t4 
                        + 5.00 * s * (_P1 * (s - 5.00 * t) * s_t3 
                          + s * t * (2.00 * _P2 * (2.00 * s - 5.00 * t) * s_t2 + s * t * (2 * _P3 * (3 * s - 5 * t) * (s - t) + s * t * (4.00 * _P4 * s - 5 * _P4 * t + _P5 * s * t)))) / (s*s*s*s*s) );
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



