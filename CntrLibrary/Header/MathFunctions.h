
/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2024 Boris Radonic

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

#pragma once
#include "CompPlatform.h"
#include "FixedPoint.h"

#ifdef HAS_ST32_CORDIC
#include "stm32g4xx_ll_cordic.h"


/* CORDIC FUNCTION: MODULUS q1.15 */
#define CORDIC_CONFIG_MODULUS   (LL_CORDIC_FUNCTION_MODULUS | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)
#endif


#include <cstdint>
#include <cmath>


namespace CntrlLibrary
{
    class MathFunctions
    {
    public:

        constexpr static int log2(uint16_t x)
        {
            return (x == 1) ? 0 : 1 + log2(x >> 1);
        }

        static std::int32_t sqrt(std::int32_t input)
        {
#ifdef HAS_ST32_CORDIC
            if (input > 0u)
            {
                std::uint32_t retVal{};
                /* Disable Irq*/
                __disable_irq();
                /* Configure CORDIC */
                WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_SQRT);
                LL_CORDIC_WriteData(CORDIC, ((std::uint32_t)input));
                /* Read sqrt and return */
                retVal = (LL_CORDIC_ReadData(CORDIC)) >> 15U;
                __enable_irq();
                return static_cast<std::int32_t>(retVal);
            }
#else

                double d = std::sqrt(static_cast<double>(input));
                float f = static_cast<float>(d);
                std::int32_t i = static_cast<std::int32_t>(f);
                return  i;
#endif
            return 0u;
         }
    };
}
