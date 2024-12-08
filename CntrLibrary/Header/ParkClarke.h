
/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/



#ifndef PARK_CLARKE_H
#define PARK_CLARKE_H

#include <cstdint>
#include <numbers>

#include "MathFunctions.h"
#include "TrigFunctions.h"


namespace CntrlLibrary
{
    // Input structure for stator values in ab_t format
    template <typename T>
    struct AB_t
    {
        AB_t() = default;

        T a{ 0 }; // Stator value along axis 'a'
        T b{ 0 }; // Stator value along axis 'b'

        constexpr bool operator==(const T& other) const
        {
            return ( (a == other.a) && (b == other.b) );
        }

    };

   
    // Output structure for values in alphabeta_t format
    template <typename T>
    struct AlphaBeta_t
    {
        AlphaBeta_t() = default;

        T alpha{ 0 }; // Transformed value along alpha axis
        T beta{ 0 };  // Transformed value along beta axis
    };

    // Output structure for q-d values
    template <typename T>
    struct QD_t
    {
        QD_t() = default;


        QD_t(const QD_t& other)
        {
            *this = other; // Use assignment operator
        }

        QD_t& operator=(const QD_t& other)
        {
            if (this != &other) { // Check for self-assignment
                this->q = other.q;
                this->d = other.d;
            }
            return *this; // Return *this to allow chaining
        }


        T q{0}; // q-axis value
        T d{0}; // d-axis value
    };


    template <typename T>
    class ParkClarke
    {
    public:

        /**
         * @brief  Perform Clarke transformation: converts stator values 'a' and 'b'
         *         into stationary reference frame values 'alpha' and 'beta'.
         * @param  input: Stator values in AB_t format (a and b).
         * @retval AlphaBeta_t: Stator values in alpha-beta stationary frame.
         */
        
        static AlphaBeta_t<T> Clarke(const AB_t<T>& input)
        {
            AlphaBeta_t<T> output{};
            output.alpha = input.a;
            T temp1 = input.a * T(std::numbers::inv_sqrt3_v<float>);
            T temp2 = input.b * T(2.0f * std::numbers::inv_sqrt3_v<float>);                
            output.beta = T(FixedPointUtils<q31_t>::QADD(temp1.raw(), temp2.raw()));
            return output;
        }
      
        /**
        * @brief  Perform Inverse Clarke Transformation: converts stationary reference
        *         frame values 'alpha' and 'beta' back into three-phase stator values 'a', 'b', and 'c'.
        * @param  input: AlphaBeta_t input with alpha and beta values.
        * @retval ABCT_t: Stator values in three-phase system (a, b, c).
        */
        static AB_t<T> InverseClarke(const AlphaBeta_t<T>& input)
        {
            AB_t<T> output;            
            // a = alpha
            output.a = input.alpha;
            T temp1 = input.alpha * T( -1.0f / 2.0f);
            T temp2 = input.beta  * T(std::numbers::sqrt3_v<float> / 2.0f);
            output.b = T(FixedPointUtils<q31_t>::QADD(temp1.raw(), temp2.raw()));
            return output;
        }


        /**
        * @brief Perform Park Transformation: converts alpha-beta stationary reference frame
        *        values to a rotating d-q synchronous reference frame.
        * @param  input: AlphaBeta_t input with alpha and beta values.
        * @param  trigComp:Sin and Cos components of electrical angle in q1.15 fixed-point format (Q15).
        * @retval QD_t: Transformed values in d-q reference frame.
        */
        static QD_t<T> Park(const AlphaBeta_t<T>& input, TrigComponents trigComp )
        {
            QD_t<T> output;
            T temp1 = input.alpha * T(trigComp.hCos);
            T temp2 = input.beta * T(trigComp.hSin);
            T temp3 = input.alpha * T(trigComp.hSin);
            T temp4 = input.beta * T(trigComp.hCos);
            output.d = T(FixedPointUtils<q31_t>::QADD(temp1.raw(), temp2.raw()));
            output.q = T(FixedPointUtils<q31_t>::QSUB(temp4.raw(), temp3.raw()));

            return output;
        }

        /**
        * @brief Perform Reverse Park Transformation: converts q-d rotating reference
        *        frame values to a stationary alpha-beta reference frame.
        * @param  input: QD_t input with q and d values.
        * @param  trigComp:Sin and Cos components of electrical angle in q1.15 fixed-point format (Q15).
        * @retval AlphaBeta_t: Transformed values in alpha-beta stationary frame.
        */
        static AlphaBeta_t<T> InvPark(QD_t<T> input, TrigComponents trigComp)
        {
            AlphaBeta_t<T> output{};
            T temp1 = input.d * trigComp.hCos;
            T temp2 = input.q * trigComp.hSin;
            T temp3 = input.d * trigComp.hSin;
            T temp4 = input.q * trigComp.hCos;
            output.alpha = T(FixedPointUtils<q31_t>::QSUB(temp1.raw(), temp2.raw()));
            output.beta  = T(FixedPointUtils<q31_t>::QADD(temp4.raw(), temp3.raw()));
            return output;
        }

    private:
        /**
         * @brief  Saturate a Q15 value to the q1.15 range.
         * @param  value: Q15 value to saturate.
         * @retval Saturated Q15 value.
         */
        static Q15 saturate(Q15 value)
        {
            return Q15( Q15::saturate( static_cast<std::int32_t>(value.raw())) );
        }
    };
}

#endif // PARK_CLARKE_H