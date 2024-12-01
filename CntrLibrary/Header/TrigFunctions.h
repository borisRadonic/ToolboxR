
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

#pragma once

#include "FixedPoint.h"

#include <cstdint>
#include <cstdint>
#include <cmath>
#include <numbers>

namespace CntrlLibrary
{
    struct TrigComponents
    {
        Q15 hCos{0.0f};
        Q15 hSin{0.0f};
    };

    class TrigFunctions
    {
        public:
            /*normed to 1*/
            static TrigComponents TrigFuncs(Q15 hAngle)
            {
            
    #ifdef HAS_ST32_CORDIC 
                /* MISRAC2012-violation Rule 19.2. The union keyword should not be used.
               * This needs to be determined:
               * Padding — how much padding is inserted at the end of the union;
               * Alignment — how are members of any structures within the union aligned;
               * Endianness — is the most significant byte of a word stored at the lowest or highest memory address;
               * Bit-order — how are bits numbered within bytes and how are bits allocated to bit fields.
               * Low. Use of union (u32toi16x2). */
                union u32toi16x2
                {
                    uint32_t CordicRdata;
                    TrigComponents Components;
                } CosSin;
                WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
                LL_CORDIC_WriteData(CORDIC, ((uint32_t)0x7FFF0000) + (static_cast<uint32_t>(hAngle.raw()));
                CosSin.CordicRdata = LL_CORDIC_ReadData(CORDIC);
                return (CosSin.Components); //cstat !UNION-type-punning
    #else
                TrigComponents components;
                components.hCos = Q15(cos(hAngle.toFloat() * static_cast<float>(std::numbers::pi)));
                components.hSin = Q15(sin(hAngle.toFloat() * static_cast<float>(std::numbers::pi)));
                return components;
    #endif
            }
    };
}
