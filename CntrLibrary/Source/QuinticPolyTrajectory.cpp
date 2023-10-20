#include "QuinticPolyTrajectory.h"
#include <functional> 
#include <cmath>
#include <limits>
//#include <Eigen/Dense>


#define MIN_VEL_NUMERIC 1e-5
#define MIN_ACCEL_NUMERIC 1e-5
#define MIN_TIME_DIFFERENCE 1e-5 /*10 us*/
#define MIN_PATH_PARTIAL_TIME 0.05e-3 /*500 us*/

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {

        QuinticPolyTrajectory::QuinticPolyTrajectory( double i_pos,
                                                      double i_vel,
                                                      double i_accel,
                                                      double f_pos,
                                                      double f_vel,
                                                      double f_accel,
                                                      double f_time)
        {
            initial_position        = i_pos;
            initial_velocity        = i_vel;
            initial_acceleration    = i_accel;
            target_position         = f_pos;
            target_velocity         = f_vel;
            target_acceleration     = f_accel;


            double a0 = 0.00;
            double a1 = 0.00;
            double a2 = 0.00;
            double a3 = 0.00;
            double a4 = 0.00;
            double a5 = 0.00;

            a0 = initial_position;
            a1 = initial_velocity;;
            a2 = initial_acceleration / 2.0;

            _tf = f_time;

            double tf_2 = _tf * _tf;
            double tf_3 = tf_2 * _tf;
            double tf_4 = tf_3 * _tf;
            double tf_5 = tf_4 * _tf;
            double tf_6 = tf_5 * _tf;
            double tf_7 = tf_6 * _tf;
            double tf_8 = tf_7 * _tf;
            /*
            Eigen::Matrix3d A;

            A << tf_3,          tf_4,           tf_5,
                 3.00 * tf_2,   4.00 * tf_3,    5.00 * tf_4,
                 6.00 * _tf,    12.00 * tf_2,   20.00 * tf_3;

            Eigen::Vector3d b(  f_pos - i_pos - i_vel * _tf - 0.50 * i_accel * _tf * _tf,
                                f_vel - i_vel - i_accel * _tf,
                                f_accel - i_accel);

            // Solve for x using Eigen's linear solver
            Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

            _a3 = x(0);
            _a4 = x(1);
            _a5 = x(2);
            */
            /*This was prooved nusing solver*/
            a3  = (20.0 * target_position - 20.0 * initial_position - (8.00 * target_velocity + 12.0 * initial_velocity) * _tf - (3.00 * initial_acceleration - target_acceleration) * tf_2) / (2.00 * tf_3);
            a4  = (30.0 * initial_position - 30.0 * target_position + ( 14.00 * target_velocity + 16.0 * initial_velocity) * _tf + (3.00 * initial_acceleration - 2.0 * target_acceleration) * tf_2) / (2.00 * tf_4);;
            a5= (12.0 * target_position - 12.0 * initial_position - (6.00 * target_velocity + 6.0 * initial_velocity) * _tf - (initial_acceleration - target_acceleration) * tf_2) / (2.00 * tf_5);;

            poly.setParams(a0, a1, a2, a3, a4, a5);

            // v(t) = _a1​ + 2 * _a2 * ​t + 3 * a3 * ​t^2 + 4 * _a4​ * t ^ 3 + 5 * _a5 * t ^ 4 -> velocity of polinomial
            // a(t) =  2 * _a2 + 6 * a3 * ​t + 12 * _a4​ * t ^ 2 + 20 * _a5 * t ^ 3 -> acceleration of polinomial
            // j(t)=   6 * _a3​ + 24 * a4 * ​t + 60 * a5 ​* t ^ 2 -> jerk of polinomial
            // s(t)=   24 * a4 + 120 * a5 * t -> Snap (The first derivative of jerk with respect to time) of polinomial
        }

        Math::BasicNumMethods::ResultType QuinticPolyTrajectory::findExtremaNewtonRaphson(std::pair<double, double>& ex_velocity, std::pair<std::pair<double, double>, std::pair<double, double>>& ex_accelerations)
        {
            Math::BasicNumMethods::ResultType result = Math::BasicNumMethods::ResultType::NotPossible;
            
            double a0 = poly.getA0();
            double a1 = poly.getA1();
            double a2 = poly.getA2();
            double a3 = poly.getA3();
            double a4 = poly.getA4();
            double a5 = poly.getA5();
            
            // Define the function of velocity
            std::function<double(double)> funcVel = [a0,a1,a2,a3,a4,a5](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x_2 * x;
                double x_4 = x_3 * x;
                return (a1 + 2.0 * a2 * x + 3.0 * a3 * x_2 + 4.00 * a4 * x_3 + 5 * a5 * x_4);
            };

            // Define the function of acceleration
            std::function<double(double)> funcAccel = [a2, a3, a4, a5](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x * x * x;
                return (2.0 * a2 + 6.0 * a3 * x + 12.00 * a4 * x_2 + 20.0 * a5 * x_3);
            };

            // Define the function of jerk
            std::function<double(double)> funcJerk = [a3, a4, a5](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x * x * x;
                return (6.0 * a3 + 24.00 * a4 * x + 60.0 * a5 * x * x);
            };

            // Define the function of snap
            std::function<double(double)> funcSnap = [a4, a5](double x) -> double
            {
                return (24.00 * a4 + 120.0 * a5 * x);
            };
            double step = _tf / 16.00;

            std::vector<double> rootsAccel;
            std::vector<double> rootsJerk;

            NewtonRaphson solverAccel(funcAccel, funcJerk, MIN_TIME_DIFFERENCE, 20);
            double root = 0.00;
            double initial_guess = 0.00;

            while (initial_guess <= (_tf - step))
            {
                result = solverAccel.findRoot(initial_guess, root);
                if (result == Math::BasicNumMethods::ResultType::Ok)
                {
                    //we do not need the start of interval where t = 0 where the initial_velocity is defined 
                    if (abs(root) > MIN_TIME_DIFFERENCE)
                    {
                        //we do not need last point where the target_velocity is defined
                        if (root < (_tf - MIN_TIME_DIFFERENCE))
                        {
                            rootsAccel.push_back(root);
                        }
                    }
                }                       
                initial_guess += step;
            }

            result = Math::BasicNumMethods::ResultType::NotPossible;
            NewtonRaphson solverJerk(funcJerk, funcSnap, MIN_TIME_DIFFERENCE, 20);
            root = 0.00;
            initial_guess = 0.00;
            while (initial_guess <= (_tf - step))
            {
                result = solverJerk.findRoot(initial_guess, root);
                if (result == Math::BasicNumMethods::ResultType::Ok)
                {
                    if (false == (abs(root) <= std::numeric_limits<double>::min()))
                    {
                        rootsJerk.push_back(root);
                    }
                }               
                initial_guess += step;
            }
            std::sort(rootsAccel.begin(), rootsAccel.end());
            std::sort(rootsJerk.begin(), rootsJerk.end());

            std::function<bool(double, double, double)> isClose = [this](double a, double b, double min) -> bool
            {
                return std::abs(a - b) <= min;
            };

            //remove values that are close to each other based on a certain tolerance
            auto endAccel = std::unique(rootsAccel.begin(), rootsAccel.end(), [&](double a, double b) { return isClose(a, b, MIN_PATH_PARTIAL_TIME); });
            rootsAccel.erase(endAccel, rootsAccel.end());

            auto endJerk = std::unique(rootsJerk.begin(), rootsJerk.end(), [&](double a, double b) { return isClose(a, b, MIN_PATH_PARTIAL_TIME); });
            rootsJerk.erase(endJerk, rootsJerk.end());

            //find max acceleration
            //this seams to be the shortest method to find MAX.acceleration values (commented at the moment )
            //double aa = 60.00 * a5;
            //double bb = 24.00 * a4;
            //double cc = 6.00 * a3;
            //double t_max_accel1 = (-bb + sqrt(bb * bb - 4.00 * aa * cc)) / (2.00 * aa);
            //double t_max_accel2 = (-bb - sqrt(bb * bb - 4.00 * aa * cc)) / (2.00 * aa);

            //
            //bool max = isMaxAccel(tf, t_max_accel1) || isMaxAccel(tf, t_max_accel2);

            uint32_t counter = 0U;
            for (auto& timea : rootsJerk)
            {
                if (counter == 0U)
                {
                    ex_accelerations.first.first = funcAccel(timea);
                    ex_accelerations.first.second = timea;
                }
                if (counter == 1U)
                {
                    ex_accelerations.second.first = funcAccel(timea);
                    ex_accelerations.second.second = timea;
                }
                if (counter > 1U)
                {
                    break;
                }
                counter++;
            }
            
            bool velToHigh = false;
            bool hasNegVel = false;
            bool hasPosVel = false;
            for (auto& timev : rootsAccel)
            {
                double maxVel = funcVel(timev);
                if (abs(maxVel) > MIN_VEL_NUMERIC)
                {
                    ex_velocity.first = maxVel;
                    ex_velocity.second = timev;
                    break;
                }
            }           
            return Math::BasicNumMethods::ResultType::Ok;
        }

        void QuinticPolyTrajectory::calculateValuesForTime(double t, double& position, double& velocity, double& acceleration, double& jerk)
        {
            if (t > 0.00 && t <= _tf)
            {
                poly.calculate(t, position, velocity, acceleration, jerk);
            }
        }
    }
}










