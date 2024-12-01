
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

#include "MathFunctions.h"
#include "TrigFunctions.h"

namespace CntrlLibrary
{
       


    // Input structure for stator values in ab_t format
    struct AB_t
    {
        AB_t() = default;

        Q15 a{ 0.0f }; // Stator value along axis 'a'
        Q15 b{ 0.0f }; // Stator value along axis 'b'
    };


    // Input structure for stator values in abc_t format
    struct ABC_t
    {
        ABC_t() = default;

        Q15 a{ 0.0f }; // Stator value along axis 'a'
        Q15 b{ 0.0f }; // Stator value along axis 'b'
        Q15 c{ 0.0f }; // Stator value along axis 'c'
    };

    // Output structure for values in alphabeta_t format
    struct AlphaBeta_t
    {
        AlphaBeta_t() = default;

        Q15 alpha{ 0.0f }; // Transformed value along alpha axis
        Q15 beta{ 0.0f };  // Transformed value along beta axis
    };

    // Output structure for q-d values
    struct QD_t
    {
        Q15 q{ 0.0f }; // q-axis value
        Q15 d{ 0.0f }; // d-axis value
    };


    class ParkClarke
    {
    public:
        static constexpr Q15 SQRT3_DIV = Q15( 1.0f / 1.73205080757f ); // 1/sqrt(3)

        /**
         * @brief  Perform Clarke transformation: converts stator values 'a' and 'b'
         *         into stationary reference frame values 'alpha' and 'beta'.
         * @param  input: Stator values in AB_t format (a and b).
         * @retval AlphaBeta_t: Stator values in alpha-beta stationary frame.
         */
        static AlphaBeta_t Clarke(const AB_t& input)
        {
            AlphaBeta_t output;

            // alpha = a
            output.alpha = input.a;

            // beta = -(2*b + a)/sqrt(3)
            Q15 a_divSQRT3 = input.a * SQRT3_DIV;
            Q15 b_divSQRT3 = input.b * SQRT3_DIV;
            Q15 beta = Q15(0.0f) - (a_divSQRT3 + (b_divSQRT3 * Q15(2.0f)));

            // Saturate beta to q1.15 range
            output.beta = saturate(beta);

            // Ensure beta is not -32768
            if (output.beta.raw() == Q15::MinValue)
            {
                output.beta = Q15( Q15::MinValue) + Q15(1.0f / 32768.0f); // Set to -32767
            }

            return output;
        }


        /**
        * @brief  Perform Inverse Clarke Transformation: converts stationary reference
        *         frame values 'alpha' and 'beta' back into three-phase stator values 'a', 'b', and 'c'.
        * @param  input: AlphaBeta_t input with alpha and beta values.
        * @retval ABCT_t: Stator values in three-phase system (a, b, c).
        */
        static ABC_t InverseClarke(const AlphaBeta_t& input)
        {
            ABC_t output;

            // a = alpha
            output.a = input.alpha;

            // b = -alpha/2 + sqrt(3)/2 * beta
            Q15 alpha_neg_half = input.alpha * Q15(-0.5f);
            Q15 beta_sqrt3_half = input.beta * Q15(0.86602540378f); // sqrt(3)/2
            output.b = alpha_neg_half + beta_sqrt3_half;

            // c = -alpha/2 - sqrt(3)/2 * beta
            Q15 beta_neg_sqrt3_half = input.beta * Q15(-0.86602540378f); // -sqrt(3)/2
            output.c = alpha_neg_half + beta_neg_sqrt3_half;

            return output;
        }


        /**
        * @brief Perform Park Transformation: converts alpha-beta stationary reference frame
        *        values to a rotating d-q synchronous reference frame.
        * @param  input: AlphaBeta_t input with alpha and beta values.
        * @param  theta: Rotation angle in q1.15 fixed-point format (Q15).
        * @retval QD_t: Transformed values in d-q reference frame.
        */
        static QD_t Park(const AlphaBeta_t& input, Q15 theta)
        {
            QD_t output;
            // Compute sin and cos components
            TrigComponents trig = TrigFunctions::TrigFuncs(theta);

            // Compute q component
            Q15 qTmp1 = input.alpha * trig.hCos;
            Q15 qTmp2 = input.beta * trig.hSin;
            Q15 resQ = qTmp1 - qTmp2;
            output.q = saturate(resQ);

            // Ensure q is not -32768
            if (output.q.raw() == Q15::MinValue)
            {
                output.q = Q15( Q15::MinValue ) + Q15( static_cast<std::int32_t>(1) );
            }
                     
            Q15 dTmp1 = input.alpha * trig.hSin;
            Q15 dTmp2 = input.beta * trig.hCos;
            Q15 resD = dTmp1 - dTmp2;
            output.d = saturate(resD);

            // Ensure q is not -32768
            if (output.d.raw() == Q15::MinValue)
            {
                output.d = Q15(Q15::MinValue) + Q15(static_cast<std::int32_t>(1));
            }
            return output;
        }

        /**
        * @brief Perform Reverse Park Transformation: converts q-d rotating reference
        *        frame values to a stationary alpha-beta reference frame.
        * @param  input: QD_t input with q and d values.
        * @param  theta: Rotation angle in q1.15 fixed-point format (Q15).
        * @retval AlphaBeta_t: Transformed values in alpha-beta stationary frame.
        */
        AlphaBeta_t InvPark(QD_t input, Q15 theta)
        {

            TrigComponents trig = TrigFunctions::TrigFuncs(theta);;
            AlphaBeta_t output;

            Q15 alpha_tmp1 = input.q * trig.hCos;
            Q15 alpha_tmp2 = input.d * trig.hSin;

            output.alpha = saturate(alpha_tmp1 + alpha_tmp2);

            Q15 beta_tmp1 = input.q * trig.hSin;
            Q15 beta_tmp2 = input.d * trig.hCos;

            output.beta = saturate(beta_tmp1 + beta_tmp2);

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