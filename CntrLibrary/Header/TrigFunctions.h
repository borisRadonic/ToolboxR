
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

#include "FixedPoint.h"

#include <cstdint>
#include <cstdint>
#include <cmath>
#include "CompPlatform.h"
#include <numbers>
#include "MathFunctions.h"

#ifdef HAS_ST32_CORDIC
#include "stm32g4xx_ll_cordic.h"


/* CORDIC FUNCTION: PHASE q1.31 (Electrical Angle computation) */
#define CORDIC_CONFIG_PHASE     (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 |\
LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)


/* CORDIC FUNCTION: COSINE q1.15 */
#define CORDIC_CONFIG_COSINE    (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)

#endif

namespace CntrlLibrary
{
    struct TrigComponents
    {
        TrigComponents()
        {
        }

        TrigComponents(const std::uint32_t data)
        {
            // Extract the lower 16 bits for hCos
            this->hCos = Q15( static_cast<std::int32_t>(static_cast<std::int16_t>(data & 0xFFFF)));
            // Extract the upper 16 bits for hSin
            this->hSin = Q15( static_cast<std::int32_t>(static_cast<std::int16_t>((data >> 16) & 0xFFFF)));
        }
        Q15 hCos{0.0f};
        Q15 hSin{0.0f};
    };

    class TrigFunctions
    {
        public:
            static __FORCEINLINE TrigComponents TrigFuncs(Q15 hAngle)
            {
    #ifdef HAS_ST32_CORDIC
                WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
                LL_CORDIC_WriteData(CORDIC, ((uint32_t)0x7FFF0000) + (static_cast<uint32_t>(hAngle.raw())));
                std::uint32_t data = LL_CORDIC_ReadData(CORDIC);
                return TrigComponents(data);
    #else
                TrigComponents components;
                components.hCos = Q15(cos(hAngle.toFloat() * std::numbers::pi_v<float>));
                components.hSin = Q15(sin(hAngle.toFloat() * std::numbers::pi_v<float>));
                return components;
    #endif
            }
    };
}
