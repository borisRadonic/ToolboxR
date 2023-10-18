#include "QuinticPolyTrajectory.h"
#include "BasicNumMethods.h"
#include <functional> 
#include <cmath>
#include <limits>


#define MIN_VEL_NUMERIC 1e-5
#define MIN_ACCEL_NUMERIC 1e-5
#define MIN_TIME_DIFFERENCE 1e-5 /*10 us*/
#define MIN_PATH_PARTIAL_TIME 0.05e-3 /*500 us*/

namespace CntrlLibrary
{
    namespace TrajectoryGeneration
    {

        QuinticPolyTrajectory::QuinticPolyTrajectory()
        {
        }
               
        bool QuinticPolyTrajectory::isMaxAccel(double tMax, double ta)
        {

            if ( (ta <= tMax) && (ta >= 0.00) )
            {               
                    double accel_m = 2.00 * _a2 + 6.00 * _a3 * ta
                        + 12.00 * _a4 * ta * ta
                        + 20.00 * _a5 * ta * ta * ta;
                    if (std::abs(accel_m) > max_acceleration)
                    {
                        return true;
                    }                
            }
            return false;
        }

        bool QuinticPolyTrajectory::prepare(double tf)
        {
            //For Quintic Polynomials
            using namespace Math;
            using namespace BasicNumMethods;
            
    
            _a0 = initial_position;
            _a1 = initial_velocity;;
            _a2 = initial_acceleration / 2.0;

            _tf = tf;

            double tf_2 = tf * tf;
            double tf_3 = tf_2 * tf;
            double tf_4 = tf_3 * tf;
            double tf_5 = tf_4 * tf;
            double tf_6 = tf_5 * tf;
            double tf_7 = tf_6 * tf;
            double tf_8 = tf_7 * tf;

            _a3 = (20.0 * target_position - 20.0 * initial_position - (8.00 * target_velocity + 12.0 * initial_velocity) * tf - (3.00 * initial_acceleration - target_acceleration) * tf_2) / (2.00 * tf_3);
            _a4 = (30.0 * initial_position - 30.0 * target_position - (14.00 * target_velocity + 16.0 * initial_velocity) * tf - (3.00 * initial_acceleration - 2.0 * target_acceleration) * tf_2) / (2.00 * tf_4);;
            _a5 = (12.0 * target_position - 12.0 * initial_position - (6.00 * target_velocity + 6.0 * initial_velocity) * tf - (initial_acceleration - target_acceleration) * tf_2) / (2.00 * tf_5);;
            
            // v(t) = _a1​ + 2 * _a2 * ​t + 3 * a3 * ​t^2 + 4 * _a4​ * t ^ 3 + 5 * _a5 * t ^ 4 -> velocity of polinomial
            // a(t) =  2 * _a2 + 6 * a3 * ​t + 12 * _a4​ * t ^ 2 + 20 * _a5 * t ^ 3 -> acceleration of polinomial
            // j(t)=   6 * _a3​ + 24 * a4 * ​t + 60 * a5 ​* t ^ 2 -> jerk of polinomial
            // s(t)=   24 * a4 + 120 * a5 * t -> Snap (The first derivative of jerk with respect to time) of polinomial

            //find max velocity finding root of acceleration

            // Define the function of velocity
            std::function<double(double)> funcVel = [this](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x_2 * x;
                double x_4 = x_3 * x;
                return (_a1 + 2.0 * _a2 * x + 3.0 * _a3 * x_2  + 4.00 * _a4 * x_3 + 5 * _a5 * x_4);
            };

            // Define the function of acceleration
            std::function<double(double)> funcAccel = [this](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x * x * x;
                return ( 2.0 * _a2 + 6.0 * _a3 * x + 12.00 * _a4 * x_2 + 20.0 * _a5 * x_3);
            };

            // Define the function of jerk
            std::function<double(double)> funcJerk = [this](double x) -> double
            {
                double x_2 = x * x;
                double x_3 = x * x * x;
                return (6.0 * _a3 + 24.00 * _a4 * x + 60.0 * _a5 * x * x);
            };

            // Define the function of snap
            std::function<double(double)> funcSnap = [this](double x) -> double
            {
                return (24.00 * _a4 + 120.0 * _a5 * x);
            };
            double step = tf / 16.00;

            std::vector<double> rootsAccel;
            std::vector<double> rootsJerk;
            std::vector<double> maxAccels;
            std::vector<double> maxVels;

            NewtonRaphson solverAccel(funcAccel, funcJerk, MIN_TIME_DIFFERENCE, 20);
            double root = 0.00;
            double initial_guess = 0.00;

            while (initial_guess <= (tf - step))
            {
                if (solverAccel.findRoot(initial_guess, root))
                {
                    //we do not need the start of interval where t = 0 where the initial_velocity is defined 
                    if (abs(root) > MIN_TIME_DIFFERENCE )
                    {
                        //we do not need last point where the target_velocity is defined
                        if (root < (tf - MIN_TIME_DIFFERENCE) )
                        {
                            rootsAccel.push_back(root);
                        }                       
                    }
                }
                initial_guess += step;
            }

            NewtonRaphson solverJerk(funcJerk, funcSnap, MIN_TIME_DIFFERENCE, 20);
            root = 0.00;
            initial_guess = 0.00;
            while (initial_guess <= (tf- step) )
            {                
                if (solverJerk.findRoot(initial_guess, root))
                {
                    if (false == (abs(root) <= std::numeric_limits<double>::min()))
                    {
                        rootsJerk.push_back( root );  
                    }
                }
                initial_guess += step;
            }
            std::sort(rootsAccel.begin(), rootsAccel.end());
            std::sort(rootsJerk.begin(), rootsJerk.end());

            std::function<bool(double,double,double)> isClose = [this] (double a,double b, double min) -> bool
            {
                return std::abs(a - b) <= min;
            };

            //remove values that are close to each other based on a certain tolerance
            auto endAccel = std::unique(rootsAccel.begin(), rootsAccel.end(), [&](double a, double b) { return isClose(a, b, MIN_PATH_PARTIAL_TIME); });
            rootsAccel.erase(endAccel, rootsAccel.end());

            auto endJerk = std::unique(rootsJerk.begin(), rootsJerk.end(), [&](double a, double b) { return isClose(a, b, MIN_PATH_PARTIAL_TIME); });
            rootsJerk.erase(endJerk, rootsJerk.end());

            //find max acceleration
            double aa = 60.00 * _a5;
            double bb = 24.00 * _a4;
            double cc = 6.00 * _a3;
            double t_max_accel1 = (-bb + sqrt(bb * bb - 4.00 * aa * cc)) / (2.00 * aa);
            double t_max_accel2 = (-bb - sqrt(bb * bb - 4.00 * aa * cc)) / (2.00 * aa);

            //this seams to be the shortest method to find MAX. acceleration values
            bool max = isMaxAccel(tf, t_max_accel1) || isMaxAccel(tf, t_max_accel2);

            for (auto& ma : rootsJerk)
            {
                maxAccels.push_back(funcAccel(ma));
            }

            bool velToHigh = false;
            bool hasNegVel = false;
            bool hasPosVel = false;
            for (auto& mv : rootsAccel)
            {
                double maxVel = funcVel(mv);
                if ( abs(maxVel) > MIN_VEL_NUMERIC )
                {
                    maxVels.push_back(maxVel);

                    if (mv < (0.00 - MIN_VEL_NUMERIC))
                    {
                        hasNegVel = true;
                    }
                    if (mv > (0.00 + MIN_VEL_NUMERIC))
                    {
                        hasPosVel = true;
                    }

                    if (abs( maxVel) > max_velocity)
                    {
                        velToHigh = true;
                    }
                }
            }

            bool accelToHigh = false;
            bool hasNegAccel = false;
            bool hasPosAccel = false;
            for (double& am : maxAccels)
            {
                if (am < (0.00 - MIN_VEL_NUMERIC))
                {
                    hasNegAccel = true;
                }
                if (am > (0.00 + MIN_VEL_NUMERIC))
                {
                    hasPosAccel = true;
                }

                if  (std::abs( am) > max_acceleration )
                {
                    accelToHigh = true;
                }
            }

           
            double t = tf;

            double scal = 0.80;
          
            std::function<double(double)> fc = [this,scal](double x) -> double
            {
                double x_2 = x * x;
                return ( std::log( 1.00 + scal *  abs(2.00 * _a2 + 6.00 * _a3 * x + 12.00 * _a4 * x_2 + 20.00 * _a5 * x * x_2)) / log(1.00 + scal));
            };

            double x1 = 0.0;
            double x2 = 1.05;
            int n = 10;  // number of intervals (must be even)

            double result = SimpsonsIntegrator::approximateIntegral(fc, x1, x2, n);
            
            return true;
        }


        void QuinticPolyTrajectory::process(double t, double& position, double& velocity, double& acceleration, double& jerk)
        {
            if (t > 0.00 && t <= _tf)
            {
                position = _a0 + _a1 * t + _a2 * t * t + _a3 * t * t * t + _a4 * t * t * t * t + _a5 * t * t * t * t * t;
                velocity = _a1 + 2.00 * _a2 * t + 3.00 * _a3 * t * t + 4.00 * _a4 * t * t * t + 5.00 * _a5 * t * t * t * t;
                acceleration = 2.00 * _a2 + 6.00 * _a3 * t + 12.00 * _a4 * t * t + 20.00 * _a5 * t * t * t;
                jerk = 6.00 * _a3 + 24.00 * _a4 * t + 60.00 * _a5 * t * t ;

               

                double u = 0.1;
                if (velocity > 0.00001)
                {
                   // velocity = std::log(1.00 + u * abs(velocity)) / log(1.00 + u);
                }
                else if(velocity < -0.00001)
                {
                    //velocity = - std::log(1.00 + u * abs(velocity)) / log(1.00 + u);
                }
            }
        }
    }
}










