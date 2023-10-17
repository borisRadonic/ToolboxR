#include "QuinticPolyTrajectory.h"
#include <cmath>


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
                    if (abs(accel_m) > max_acceleration)
                    {
                        return true;
                    }                
            }
            return false;
        }

        bool QuinticPolyTrajectory::prepare(double tf)
        {
            //For Quintic Polynomials

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
            // j(t)=   6 * _a3​ + 24 * a4 * ​t + 60 * a5 ​t2t ^ 2 -> jerk of polinomial

            //find max acceleration
            double aa = 60.00 * _a5;
            double bb = 24.00 * _a4;
            double cc = 6.00 * _a3;
            double t_max_accel1 = (-bb + sqrt(bb * bb - 4.00 * aa * cc)) / (2.00 * aa);
            double t_max_accel2 = (-bb - sqrt(bb * bb - 4.00 * aa * cc)) / (2.00 * aa);

            bool max = isMaxAccel(tf, t_max_accel1) || isMaxAccel(tf, t_max_accel2);
 
            return true;
        }


        void QuinticPolyTrajectory::process(double t, double& position, double& velocity, double& acceleration)
        {
            if (t > 0.00 && t <= _tf)
            {
                position = _a0 + _a1 * t + _a2 * t * t + _a3 * t * t * t + _a4 * t * t * t * t + _a5 * t * t * t * t * t;
                velocity = _a1 + 2.00 * _a2 * t + 3.00 * _a3 * t * t + 4.00 * _a4 * t * t * t + 5.00 * _a5 * t * t * t * t;
                acceleration = 2.00 * _a2 + 6.00 * _a3 * t + 12.00 * _a4 * t * t + 20.00 * _a5 * t * t * t;
            }
        }
    }
}










